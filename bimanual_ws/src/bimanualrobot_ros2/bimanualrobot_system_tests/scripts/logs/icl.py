#!/usr/bin/env python3
"""
ICL Query Script for Ping Pong Ball Bouncing
Loads demonstrations, builds prompt, queries OpenAI API, returns DMP weights.
"""

import json
import numpy as np
from pathlib import Path
from openai import OpenAI
import os

# ============================================================
# CONFIGURATION
# ============================================================

LOGS_DIR = Path("src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs/good")
EPISODES_FILE = LOGS_DIR / "processed_episodes.json"
BASELINE_FILE = LOGS_DIR / "baseline.npz"  # NEW: Load shared y0*, g*

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
MODEL = "gpt-4o"  # or "gpt-4-turbo"

NUM_GOOD_EXAMPLES = 3
NUM_BAD_EXAMPLES = 1

# ============================================================
# HELPER FUNCTIONS
# ============================================================

def load_episodes(filepath):
    """Load processed episodes from JSON."""
    with open(filepath, 'r') as f:
        return json.load(f)

def load_baseline(filepath):
    """Load baseline DMP parameters (shared y0*, g*, basis functions)."""
    data = np.load(filepath, allow_pickle=True)
    return {
        'y0_star': data.get('y0_star', None),  # May not exist if old baseline
        'g_star': data.get('g_star', None),
        'c': data['c'],
        'h': data['h'],
        'K': float(data['K']),
        'D': float(data['D']),
        'alpha_s': float(data['alpha_s']),
        'run_time': float(data['run_time']),
        'M': len(data['c'])
    }

def load_dmp_weights(episode_id, logs_dir):
    """
    Load DMP weights for an episode.
    File format: wrist_{episode_id}_weights.npz
    Contains: w (shape 3 x M), y0, g
    """
    weight_file = logs_dir / f"wrist_{episode_id}_weights.npz"
    
    if not weight_file.exists():
        print(f"Warning: DMP weights not found for {episode_id}")
        return None
    
    try:
        data = np.load(weight_file)
        # Shape: (3, M) - one row per axis (x, y, z)
        weights = data['w']
        y0 = data['y0']
        g = data['g']
        return {
            'weights': weights,
            'y0': y0,
            'g': g
        }
    except Exception as e:
        print(f"Error loading DMP weights for {episode_id}: {e}")
        return None

def format_vector(vec, precision=3):
    """Format numpy array or list as string."""
    if isinstance(vec, np.ndarray):
        vec = vec.tolist()
    return f"[{', '.join(f'{x:.{precision}f}' for x in vec)}]"

def format_dmp_weights_compact(weights, n_preview=5):
    """
    Show compact preview of DMP weights.
    weights shape: (3, M) -> show first n_preview of each axis
    """
    if weights is None:
        return "[weights not available]"
    
    # Flatten to 1D for display
    flat = weights.flatten()
    M = weights.shape[1]
    
    preview = flat[:n_preview]
    preview_str = ', '.join(f'{w:.3f}' for w in preview)
    
    return f"[{preview_str}, ...] (shape: 3×{M}, total: {len(flat)} values)"

def build_icl_prompt(episodes, logs_dir, baseline, n_good=3, n_bad=1):
    """
    Build ICL prompt for DMP weight generation.
    
    Args:
        episodes: List of episode dicts (sorted by reward)
        logs_dir: Path to logs directory
        baseline: Dict with shared y0*, g*, and basis params
        n_good: Number of successful episodes to show
        n_bad: Number of failed episodes for contrast
    
    Returns:
        prompt string
    """
    
    good_episodes = episodes[:n_good]
    bad_episodes = episodes[-n_bad:] if len(episodes) > n_good else []
    
    prompt = """# Task: Ping Pong Ball Bouncing - Sequential Hit Control

## Objective
Generate DMP weights for a wrist trajectory that enables 3-4 consecutive successful ball bounces.
The ball hangs from a rope attached at (x=0.78, y=0.55, z=2.1).

## Key Constraints
- Ball must rise ≥ 10cm per hit (validates contact strength)
- Ball must stay within 50cm radius of rope anchor (prevents drift-out)
- Same motion is repeated for all hits in an episode

## Success Pattern
**Good episode:** Drift remains stable across hits [0.30, 0.28, 0.32, 0.29]m → 4 hits ✓
**Bad episode:** Drift accumulates [0.35, 0.42, 0.48, 0.53]m → 3 hits, then miss ✗

## Action Space: DMP Weights
You will generate weights that define the wrist trajectory shape.
- DMP uses goal-invariant formulation
- All episodes share canonical start (y0*) and goal (g*) positions
- **Weights encode the SHAPE of motion** in this normalized space
- Same weights are executed for all hits in an episode

"""
    
    # Show baseline info
    if baseline.get('y0_star') is not None:
        prompt += f"**Canonical Trajectory Endpoints:**\n"
        prompt += f"- Start position (y0*): {format_vector(baseline['y0_star'])}\n"
        prompt += f"- Goal position (g*): {format_vector(baseline['g_star'])}\n"
    
    prompt += f"**DMP Parameters:**\n"
    prompt += f"- Basis functions: {baseline['M']} RBFs per axis\n"
    prompt += f"- Weight dimensions: 3 axes × {baseline['M']} = {3 * baseline['M']} total values\n"
    prompt += f"- Motion duration: ~{baseline['run_time']:.2f}s\n\n"
    
    prompt += "---\n\n## Demonstration Episodes\n\n"
    
    # Add successful episodes
    for i, episode in enumerate(good_episodes):
        meta = episode['metadata']
        
        prompt += f"### Episode {i+1}: SUCCESS ({episode['num_successful_hits']}/{episode['num_hits']} hits)\n"
        prompt += f"**Episode reward:** {episode['episode_reward']:.3f}\n"
        prompt += f"**Consistency score:** {meta['consistency_score']:.3f} "
        prompt += f"(drift_std={meta['drift_std']:.3f}, drift_trend={meta['drift_trend']:+.3f})\n\n"
        
        # Load DMP weights
        dmp_data = load_dmp_weights(episode['episode_id'], logs_dir)
        
        if dmp_data:
            prompt += f"**DMP Weights (used for ALL hits):**\n"
            prompt += f"```\n{format_dmp_weights_compact(dmp_data['weights'], n_preview=8)}\n```\n\n"
        
        # Show sequential hits
        prompt += f"**Hit Sequence:**\n```\n"
        for hit in episode['hits']:
            status = "✓" if hit['success'] else "✗"
            prompt += f"Hit {hit['hit_number']} {status}: "
            prompt += f"ball_pos={format_vector(hit['ball_pos_before'], precision=2)}, "
            prompt += f"height={hit['height_gain']:.3f}m, drift={hit['max_xy_drift']:.3f}m\n"
        prompt += f"```\n\n"
        
        prompt += f"**Episode Analysis:**\n"
        prompt += f"- Average height: {meta['avg_height']:.3f}m (consistent power)\n"
        prompt += f"- Drift range: [{meta['min_drift']:.3f}, {meta['max_drift']:.3f}]m (stable control)\n"
        prompt += f"- All hits successful: {episode['all_successful']}\n\n"
        prompt += "**Why this works:** "
        
        if meta['drift_std'] < 0.05:
            prompt += "Very stable drift across hits → motion is repeatable and well-controlled. "
        if meta['avg_height'] > 0.15:
            prompt += "Good height gains → sufficient upward force. "
        if meta['drift_trend'] < 0.01:
            prompt += "Drift not accumulating → ball returns to similar position each time."
        
        prompt += "\n\n---\n\n"
    
    # Add failure examples for contrast
    if bad_episodes:
        for i, episode in enumerate(bad_episodes):
            meta = episode['metadata']
            
            prompt += f"### Counter-Example {i+1}: FAILURE ({episode['num_successful_hits']}/{episode['num_hits']} hits)\n"
            prompt += f"**Episode reward:** {episode['episode_reward']:.3f}\n"
            prompt += f"**Consistency score:** {meta['consistency_score']:.3f} "
            prompt += f"(drift_std={meta['drift_std']:.3f}, drift_trend={meta['drift_trend']:+.3f})\n\n"
            
            dmp_data = load_dmp_weights(episode['episode_id'], logs_dir)
            if dmp_data:
                prompt += f"**DMP Weights:**\n"
                prompt += f"```\n{format_dmp_weights_compact(dmp_data['weights'], n_preview=8)}\n```\n\n"
            
            # Show where it failed
            fail_hit = next((h for h in episode['hits'] if not h['success']), episode['hits'][0])
            
            prompt += f"**Failure at Hit {fail_hit['hit_number']}:**\n"
            prompt += f"- Height: {fail_hit['height_gain']:.3f}m "
            if fail_hit['height_gain'] < 0.10:
                prompt += "✗ (too weak - insufficient force)\n"
            else:
                prompt += "✓\n"
            
            prompt += f"- Drift: {fail_hit['max_xy_drift']:.3f}m "
            if fail_hit['max_xy_drift'] >= 0.50:
                prompt += "✗ (ball went too far - lost control)\n"
            else:
                prompt += "✓\n"
            
            prompt += f"\n**Why this failed:** "
            if meta['drift_trend'] > 0.05:
                prompt += "Drift accumulating (trend={:.3f}) → motion too forceful, ball progressively drifts out of range. ".format(meta['drift_trend'])
            if meta['avg_height'] < 0.10:
                prompt += "Low height gains → insufficient power, ball doesn't return reliably. "
            
            prompt += "\n\n---\n\n"
    
    # Query section
    prompt += f"""## Your Task

Generate NEW DMP weights that will achieve 3-4 consecutive successful hits with stable drift.

**Required Output Format (JSON only):**
```json
{{
  "dmp_weights": [
    [w_x1, w_x2, ..., w_x{baseline['M']}],  // X-axis weights ({baseline['M']} values)
    [w_y1, w_y2, ..., w_y{baseline['M']}],  // Y-axis weights ({baseline['M']} values)  
    [w_z1, w_z2, ..., w_z{baseline['M']}]   // Z-axis weights ({baseline['M']} values)
  ],
  "reasoning": "Brief explanation of strategy (e.g., 'gentler upward motion to reduce drift accumulation')",
  "expected_outcome": {{
    "predicted_hits": 3 or 4,
    "avg_drift_m": <predicted average drift>,
    "drift_will_be_stable": true/false,
    "confidence": <0.0 to 1.0>
  }}
}}
```

**Critical Guidelines:**
1. **Weight magnitudes:** Successful episodes typically have weights in range [-0.3, +0.3]
2. **Drift control:** Aim for drift_std < 0.05 to ensure stability across hits
3. **Consistent power:** Target heights around 0.15-0.20m per hit
4. **Shape matters:** Smooth weight curves → smoother motion → more repeatable outcomes

**Pattern Recognition:**
- Weights with large values (|w| > 0.4) → too aggressive → drift accumulates
- Weights with tiny values (|w| < 0.1) → too gentle → ball doesn't rise enough
- Erratic weight patterns → jerky motion → inconsistent results

Respond with JSON only (no markdown formatting):
"""
    
    return prompt

def query_openai(prompt, api_key=None, model=MODEL):
    """Query OpenAI API."""
    if api_key is None:
        api_key = OPENAI_API_KEY
    
    if not api_key:
        raise ValueError("OpenAI API key not set.")
    
    client = OpenAI(api_key=api_key)
    
    print("Querying OpenAI API...")
    print(f"Model: {model}")
    print(f"Prompt length: {len(prompt)} characters")
    
    response = client.chat.completions.create(
        model=model,
        messages=[
            {
                "role": "system",
                "content": "You are an expert in robot motion planning and Dynamic Movement Primitives. "
                          "You generate DMP weights based on demonstrated patterns and desired outcomes. "
                          "Always respond with valid JSON matching the requested format."
            },
            {
                "role": "user",
                "content": prompt
            }
        ],
        temperature=0.7,
        max_tokens=3000  # Increased for weight arrays
    )
    
    return response.choices[0].message.content

def parse_response(response_text):
    """Parse OpenAI response to extract DMP weights."""
    cleaned = response_text.strip()
    
    # Remove markdown code blocks
    if cleaned.startswith("```json"):
        cleaned = cleaned[7:]
    if cleaned.startswith("```"):
        cleaned = cleaned[3:]
    if cleaned.endswith("```"):
        cleaned = cleaned[:-3]
    
    cleaned = cleaned.strip()
    
    try:
        result = json.loads(cleaned)
        
        # Validate structure
        if 'dmp_weights' not in result:
            print("⚠️  Warning: No 'dmp_weights' in response")
            return None
        
        weights = np.array(result['dmp_weights'])
        
        # Validate shape
        if weights.shape[0] != 3:
            print(f"⚠️  Warning: Expected shape (3, M), got {weights.shape}")
            return None
        
        return result
        
    except json.JSONDecodeError as e:
        print(f"❌ JSON decode error: {e}")
        print(f"Response:\n{response_text[:500]}")
        return None

# ============================================================
# MAIN
# ============================================================

def main():
    print("="*70)
    print("ICL Query for Ping Pong Ball Bouncing")
    print("="*70)
    
    # Check API key
    if not OPENAI_API_KEY:
        print("\n❌ ERROR: OPENAI_API_KEY not set!")
        return
    
    # Load baseline
    print(f"\n📂 Loading baseline from: {BASELINE_FILE}")
    if not BASELINE_FILE.exists():
        print(f"❌ ERROR: Baseline file not found!")
        return
    
    baseline = load_baseline(BASELINE_FILE)
    print(f"✓ Loaded baseline (M={baseline['M']} basis functions)")
    
    # Load episodes
    print(f"\n📂 Loading episodes from: {EPISODES_FILE}")
    if not EPISODES_FILE.exists():
        print(f"❌ ERROR: Episodes file not found!")
        return
    
    episodes = load_episodes(EPISODES_FILE)
    print(f"✓ Loaded {len(episodes)} episodes")
    
    # Show top episodes
    print(f"\nTop 3 episodes:")
    for i, ep in enumerate(episodes[:3]):
        print(f"  {i+1}. {ep['episode_id']}: reward={ep['episode_reward']:.3f}, "
              f"{ep['num_successful_hits']}/{ep['num_hits']} hits, "
              f"consistency={ep['metadata']['consistency_score']:.3f}")
    
    # Build prompt
    print(f"\n📝 Building ICL prompt...")
    prompt = build_icl_prompt(
        episodes, 
        LOGS_DIR,
        baseline,
        n_good=NUM_GOOD_EXAMPLES,
        n_bad=NUM_BAD_EXAMPLES
    )
    
    # Save prompt
    prompt_file = LOGS_DIR / "last_icl_prompt.txt"
    with open(prompt_file, 'w') as f:
        f.write(prompt)
    print(f"✓ Prompt saved to: {prompt_file}")
    print(f"  Length: {len(prompt)} chars, ~{len(prompt.split())} words")
    
    # Query OpenAI
    print(f"\n🤖 Querying OpenAI ({MODEL})...")
    try:
        response_text = query_openai(prompt)
        print(f"✓ Received response")
    except Exception as e:
        print(f"❌ Error: {e}")
        return
    
    # Save response
    response_file = LOGS_DIR / "last_icl_response.txt"
    with open(response_file, 'w') as f:
        f.write(response_text)
    print(f"✓ Response saved to: {response_file}")
    
    # Parse
    print(f"\n📊 Parsing response...")
    result = parse_response(response_text)
    
    if result is None:
        print("❌ Failed to parse!")
        return
    
    # Display results
    print(f"\n{'='*70}")
    print("RESULTS")
    print(f"{'='*70}")
    
    if 'reasoning' in result:
        print(f"\n💭 Reasoning:")
        print(f"   {result['reasoning']}")
    
    if 'expected_outcome' in result:
        print(f"\n📈 Expected Outcome:")
        outcome = result['expected_outcome']
        print(f"   Predicted hits: {outcome.get('predicted_hits', 'N/A')}")
        print(f"   Avg drift: {outcome.get('avg_drift_m', 'N/A')}m")
        print(f"   Stable drift: {outcome.get('drift_will_be_stable', 'N/A')}")
        print(f"   Confidence: {outcome.get('confidence', 'N/A')}")
    
    if 'dmp_weights' in result:
        weights = np.array(result['dmp_weights'])
        print(f"\n🎯 Generated DMP Weights:")
        print(f"   Shape: {weights.shape} (3 axes × {weights.shape[1]} basis functions)")
        print(f"   Range: [{weights.min():.3f}, {weights.max():.3f}]")
        print(f"   Mean: {weights.mean():.3f}, Std: {weights.std():.3f}")
        
        for i, axis in enumerate(['X', 'Y', 'Z']):
            w = weights[i]
            print(f"   {axis}-axis: mean={w.mean():+.3f}, std={w.std():.3f}, range=[{w.min():+.3f}, {w.max():+.3f}]")
        
        # Save in DMP format (matching your dmp.py output)
        weights_file = LOGS_DIR / "generated_dmp_weights.npz"
        np.savez_compressed(
            weights_file, 
            w=weights,
            y0=baseline.get('y0_star', np.array([0.6, 0.3, 1.2])),  # Use baseline if available
            g=baseline.get('g_star', np.array([0.8, 0.55, 1.65])),
            meta=np.array({'source': 'ICL_generated'}, dtype=object)
        )
        print(f"\n✓ Weights saved to: {weights_file}")
        print(f"   Format: Compatible with your DMP rollout code")
    else:
        print("\n⚠️  No DMP weights generated!")
    
    print(f"\n{'='*70}")
    print("✓ ICL Query Complete!")
    print(f"{'='*70}\n")

if __name__ == "__main__":
    main()


