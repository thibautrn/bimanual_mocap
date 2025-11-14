import numpy as np
from pathlib import Path
import json

def load_log_file(filepath):
    """Load a log file into numpy array."""
    data = np.loadtxt(filepath, comments='#')
    return data

def find_ball_state_around_contact(ball_data, contact_time, window_before=0.1, window_after=0.5):
    """
    Extract ball state before and after a contact.
    
    Args:
        ball_data: numpy array with columns [time, x, y, z, vx, vy, vz]
        contact_time: timestamp of contact
        window_before: seconds before contact to average
        window_after: seconds after contact to analyze
    
    Returns:
        dict with ball state before/after contact
    """
    # Ball state just before contact (average over 100ms window)
    pre_mask = (ball_data[:, 0] >= contact_time - window_before) & (ball_data[:, 0] < contact_time)
    
    if np.sum(pre_mask) == 0:
        return None
    
    ball_pos_before = np.mean(ball_data[pre_mask, 1:4], axis=0)  # x, y, z
    
    # Ball trajectory after contact
    post_mask = (ball_data[:, 0] > contact_time) & (ball_data[:, 0] <= contact_time + window_after)
    
    if np.sum(post_mask) == 0:
        return None
    
    ball_trajectory_after = ball_data[post_mask, 1:4]  # xyz positions
    
    # Find maximum height reached
    z_at_contact = ball_pos_before[2]
    z_max = np.max(ball_trajectory_after[:, 2])
    height_gain = z_max - z_at_contact
    
    # Check if ball comes back down (rope physics validation)
    half_idx = len(ball_trajectory_after) // 2
    if half_idx > 0:
        z_descent = z_max - np.min(ball_trajectory_after[half_idx:, 2])
    else:
        z_descent = 0.0
    
    # Check centering (distance from rope anchor)
    rope_anchor = np.array([0.78, 0.55])
    xy_drifts = np.linalg.norm(ball_trajectory_after[:, :2] - rope_anchor, axis=1)
    max_xy_drift = np.max(xy_drifts)
    
    return {
        'ball_pos_before': ball_pos_before,
        'ball_pos_after': ball_trajectory_after[-1],
        'height_gain': height_gain,
        'z_descent': z_descent,
        'max_xy_drift': max_xy_drift,
        'z_max': z_max,
        'z_at_contact': z_at_contact
    }

def compute_hit_reward(hit_data):
    """
    Compute reward for a single hit.
    
    Criteria:
    - Ball goes up at least 10cm (lowered from 12cm)
    - Stays within 50cm of rope center
    - Returns down (validates rope physics)
    """
    if hit_data is None:
        return 0.0, {'miss': True, 'success': False}
    
    rewards = {}
    
    # 1. HEIGHT GAIN (primary metric)
    height = float(hit_data['height_gain'])
    HEIGHT_THRESHOLD = 0.10  # 10cm - easier to achieve
    HEIGHT_TARGET = 0.20     # 20cm - ideal height
    
    if height >= HEIGHT_THRESHOLD:
        # Reward heights between 10-20cm with smooth curve
        rewards['height'] = float(np.tanh(height / HEIGHT_TARGET))
    else:
        # Penalize insufficient height
        rewards['height'] = float((height / HEIGHT_THRESHOLD) - 1.0)
    
    # 2. CENTERING (stays near rope anchor)
    drift = float(hit_data['max_xy_drift'])
    DRIFT_THRESHOLD = 0.50  # 50cm max
    DRIFT_WARNING = 0.40    # Start penalizing above 40cm
    
    if drift < DRIFT_WARNING:
        # Good control
        rewards['centering'] = 1.0
    elif drift < DRIFT_THRESHOLD:
        # Acceptable but approaching limit
        rewards['centering'] = float(1.0 - ((drift - DRIFT_WARNING) / (DRIFT_THRESHOLD - DRIFT_WARNING)) * 0.5)
    else:
        # Too far - likely to miss next hit
        rewards['centering'] = -0.5
    
    # 3. RETURN DOWN (rope physics validation)
    descent = float(hit_data['z_descent'])
    DESCENT_THRESHOLD = 0.08  # 8cm - slightly lower for more lenient
    
    if descent >= DESCENT_THRESHOLD:
        rewards['descent'] = 1.0
    else:
        # Penalize if ball doesn't come back (physics issue)
        rewards['descent'] = float((descent / DESCENT_THRESHOLD) - 0.5)
    
    # TOTAL REWARD (weighted sum - removed speed component)
    weights = {
        'height': 2.0,      # Most important: can we hit the ball effectively?
        'centering': 1.5,   # Increased weight: critical for consecutive hits
        'descent': 0.5      # Decreased weight: mainly for physics validation
    }
    
    total = sum(weights[k] * rewards[k] for k in rewards.keys())
    normalized = float(total / sum(weights.values()))
    
    # Success flag
    success = bool(height >= HEIGHT_THRESHOLD and drift < DRIFT_THRESHOLD)
    
    return normalized, {**rewards, 'success': success}

def compute_drift_consistency_penalty(hits):
    """
    NEW FUNCTION: Penalize drift accumulation across hits.
    
    Good episode: drifts like [0.30, 0.28, 0.32, 0.29] - stable
    Bad episode:  drifts like [0.35, 0.42, 0.48, 0.53] - increasing
    
    Returns:
        consistency_score: 1.0 (perfect) to 0.0 (terrible)
    """
    if len(hits) < 2:
        return 1.0  # Can't judge consistency with 1 hit
    
    drifts = [h['max_xy_drift'] for h in hits]
    
    # Compute drift variance (lower is better)
    drift_std = float(np.std(drifts))
    
    # Compute drift trend (is it increasing?)
    # Positive slope = drift getting worse
    x = np.arange(len(drifts))
    slope = float(np.polyfit(x, drifts, 1)[0])
    
    # Score based on std (penalize inconsistency)
    # Good: std < 0.05 → score = 1.0
    # Bad:  std > 0.15 → score = 0.0
    std_score = max(0.0, min(1.0, 1.0 - (drift_std / 0.15)))
    
    # Score based on trend (penalize increasing drift)
    # Good: slope ≤ 0 (stable or improving) → score = 1.0
    # Bad:  slope > 0.1 (diverging fast) → score = 0.0
    if slope <= 0:
        trend_score = 1.0
    else:
        trend_score = max(0.0, 1.0 - (slope / 0.1))
    
    # Combined consistency score (both matter)
    consistency = 0.6 * std_score + 0.4 * trend_score
    
    return float(consistency)

def compute_episode_reward(hits):
    """
    Compute overall episode reward.
    
    NEW FORMULATION - Focus on:
    1. Hit count (can we chain 3-4 hits?)
    2. Hit quality (are they good hits?)
    3. Drift consistency (is motion stable across hits?)
    """
    if not hits:
        return 0.0
    
    num_hits = len(hits)
    num_successful = sum(1 for h in hits if h.get('success', False))
    
    # 1. Hit count score (0.0 to 1.0)
    # 3+ successful hits = full score
    hit_count_score = float(min(num_successful / 3.0, 1.0))
    
    # 2. Average quality of successful hits
    successful_hits = [h for h in hits if h.get('success', False)]
    if successful_hits:
        avg_quality = float(np.mean([h['reward'] for h in successful_hits]))
    else:
        # No successful hits - use average of all hits (will be negative)
        avg_quality = float(np.mean([h['reward'] for h in hits]))
    
    # 3. Drift consistency (NEW - critical for multi-hit stability)
    consistency_score = compute_drift_consistency_penalty(hits)
    
    # Final episode reward - NEW WEIGHTS
    episode_reward = float(
        0.50 * hit_count_score +      # 50%: Can we complete multiple hits?
        0.30 * avg_quality +           # 30%: Are individual hits good?
        0.20 * consistency_score       # 20%: Is the motion stable?
    )
    
    return episode_reward, consistency_score

def process_episode(log_dir, episode_timestamp):
    """
    Process one episode's log files.
    
    File formats:
    - ball: [t, x, y, z, vx, vy, vz]
    - contacts: [t, racket_x, racket_y, racket_z, ball_x, ball_y, ball_z, ball_vx, ball_vy, ball_vz, distance]
    - racket: [t, x, y, z]
    
    Args:
        log_dir: Path to logs directory
        episode_timestamp: Timestamp string (e.g., "20251108_015256")
    
    Returns:
        episode dict with all hits and rewards
    """
    # Build file paths
    ball_file = log_dir / f"ball_{episode_timestamp}.txt"
    contact_file = log_dir / f"contacts_{episode_timestamp}.txt"
    racket_file = log_dir / f"racket_{episode_timestamp}.txt"
    
    # Check required files exist
    if not ball_file.exists():
        print(f"  ✗ Missing ball file")
        return None
    if not contact_file.exists():
        print(f"  ✗ Missing contact file")
        return None
    
    # Load data
    try:
        ball_data = load_log_file(ball_file)
        contact_data = load_log_file(contact_file)
    except Exception as e:
        print(f"  ✗ Error loading files: {e}")
        return None
    
    if len(contact_data.shape) == 1:
        # Single contact, reshape to 2D
        contact_data = contact_data.reshape(1, -1)
    
    if len(contact_data) == 0:
        print(f"  ✗ No contacts detected")
        return None
    
    # Process each contact/hit
    hits = []
    for i in range(len(contact_data)):
        contact_row = contact_data[i]
        
        # Parse contact data
        contact_time = contact_row[0]
        racket_pos_at_contact = contact_row[1:4]
        ball_pos_at_contact = contact_row[4:7]
        contact_distance = contact_row[10]
        
        # Get ball state around this contact from ball trajectory log
        ball_state = find_ball_state_around_contact(ball_data, contact_time)
        
        if ball_state is None:
            print(f"  ⚠ Hit {i+1}: No ball trajectory data around t={contact_time:.2f}")
            continue
        
        # Compute reward for this hit
        hit_reward, reward_components = compute_hit_reward(ball_state)
        
        hit = {
            'hit_number': i + 1,
            'contact_time': float(contact_time),
            'contact_distance': float(contact_distance),
            'racket_pos_at_contact': racket_pos_at_contact.tolist(),
            'ball_pos_at_contact': ball_pos_at_contact.tolist(),
            'ball_pos_before': ball_state['ball_pos_before'].tolist(),
            'ball_pos_after': ball_state['ball_pos_after'].tolist(),
            'height_gain': float(ball_state['height_gain']),
            'max_xy_drift': float(ball_state['max_xy_drift']),
            'z_descent': float(ball_state['z_descent']),
            'z_max': float(ball_state['z_max']),
            'reward': float(hit_reward),
            'reward_components': {k: float(v) for k, v in reward_components.items()},
            'success': reward_components['success']
        }
        
        hits.append(hit)
        
        # Print hit info
        status = "✓" if hit['success'] else "✗"
        print(f"  {status} Hit {i+1}: h={hit['height_gain']:.3f}m, drift={hit['max_xy_drift']:.3f}m, "
              f"descent={hit['z_descent']:.3f}m, reward={hit_reward:.3f}")
    
    if not hits:
        print(f"  ✗ No valid hits found")
        return None
    
    # Compute episode-level metrics
    episode_reward, consistency_score = compute_episode_reward(hits)
    num_successful = sum(1 for h in hits if h['success'])
    
    # Compute drift statistics for analysis
    drifts = [h['max_xy_drift'] for h in hits]
    drift_std = float(np.std(drifts))
    drift_trend = float(np.polyfit(np.arange(len(drifts)), drifts, 1)[0]) if len(drifts) > 1 else 0.0
    
    episode = {
        'episode_id': episode_timestamp,
        'num_hits': int(len(hits)),
        'num_successful_hits': int(num_successful),
        'hits': hits,
        'episode_reward': float(episode_reward),
        'all_successful': bool(num_successful == len(hits)),
        'metadata': {
            'avg_height': float(np.mean([h['height_gain'] for h in hits])),
            'max_height': float(np.max([h['height_gain'] for h in hits])),
            'min_height': float(np.min([h['height_gain'] for h in hits])),
            'avg_drift': float(np.mean(drifts)),
            'max_drift': float(np.max(drifts)),
            'min_drift': float(np.min(drifts)),
            'drift_std': drift_std,
            'drift_trend': drift_trend,
            'consistency_score': float(consistency_score),
            'avg_descent': float(np.mean([h['z_descent'] for h in hits])),
            'success_rate': float(num_successful / len(hits)) if hits else 0.0
        }
    }
    
    return episode

def main():
    """Process all episodes in the log directory."""
    
    # Path to your logs directory
    log_dir = Path("src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs/good")
    
    # Make it absolute if needed
    if not log_dir.is_absolute():
        log_dir = Path.cwd() / log_dir
    
    if not log_dir.exists():
        print(f"Error: Log directory not found: {log_dir}")
        print(f"Current working directory: {Path.cwd()}")
        return
    
    # Find all unique episode timestamps by looking at ball files
    ball_files = sorted(log_dir.glob("ball_*.txt"))
    episode_timestamps = [f.stem.replace("ball_", "") for f in ball_files]
    
    print(f"Found {len(episode_timestamps)} episodes in {log_dir.name}/")
    print("="*70)
    
    episodes = []
    for timestamp in episode_timestamps:
        print(f"\nProcessing: {timestamp}")
        episode = process_episode(log_dir, timestamp)
        
        if episode is not None:
            episodes.append(episode)
            print(f"  → Episode reward: {episode['episode_reward']:.3f} "
                  f"({episode['num_successful_hits']}/{episode['num_hits']} successful hits, "
                  f"consistency: {episode['metadata']['consistency_score']:.3f})")
    
    if not episodes:
        print("\nNo valid episodes found!")
        return
    
    # Sort by episode reward (best first)
    episodes.sort(key=lambda x: x['episode_reward'], reverse=True)
    
    # Save processed episodes
    output_file = log_dir / "processed_episodes.json"
    with open(output_file, 'w') as f:
        json.dump(episodes, f, indent=2)
    
    # Print summary
    print(f"\n{'='*70}")
    print(f"SUMMARY")
    print(f"{'='*70}")
    print(f"Total episodes processed: {len(episodes)}")
    
    # Statistics
    total_hits = sum(ep['num_hits'] for ep in episodes)
    total_successful = sum(ep['num_successful_hits'] for ep in episodes)
    print(f"Total hits: {total_hits} ({total_successful} successful, {100*total_successful/total_hits:.1f}%)")
    
    # Consistency analysis
    avg_consistency = np.mean([ep['metadata']['consistency_score'] for ep in episodes])
    print(f"Average consistency score: {avg_consistency:.3f}")
    
    print(f"\nTop 5 episodes (sorted by reward):")
    for i, ep in enumerate(episodes[:min(5, len(episodes))]):
        meta = ep['metadata']
        print(f"  {i+1}. {ep['episode_id']}: reward={ep['episode_reward']:.3f}")
        print(f"     Hits: {ep['num_successful_hits']}/{ep['num_hits']} successful")
        print(f"     Height: avg={meta['avg_height']:.3f}m, max={meta['max_height']:.3f}m")
        print(f"     Drift: avg={meta['avg_drift']:.3f}m, std={meta['drift_std']:.3f}, trend={meta['drift_trend']:+.3f}")
        print(f"     Consistency: {meta['consistency_score']:.3f}")
    
    if len(episodes) > 5:
        print(f"\nBottom 3 episodes:")
        for i, ep in enumerate(episodes[-3:]):
            idx = len(episodes) - 3 + i + 1
            meta = ep['metadata']
            print(f"  {idx}. {ep['episode_id']}: reward={ep['episode_reward']:.3f}, "
                  f"{ep['num_successful_hits']}/{ep['num_hits']} hits, "
                  f"drift_std={meta['drift_std']:.3f}")
    
    # Analysis: what separates good from bad episodes?
    print(f"\n{'='*70}")
    print("PATTERN ANALYSIS")
    print(f"{'='*70}")
    
    top_3 = episodes[:3]
    bottom_3 = episodes[-3:]
    
    print(f"\nTop 3 episodes characteristics:")
    for key in ['avg_drift', 'drift_std', 'drift_trend', 'avg_height', 'consistency_score']:
        values = [ep['metadata'][key] for ep in top_3]
        print(f"  {key}: {np.mean(values):.3f} ± {np.std(values):.3f}")
    
    print(f"\nBottom 3 episodes characteristics:")
    for key in ['avg_drift', 'drift_std', 'drift_trend', 'avg_height', 'consistency_score']:
        values = [ep['metadata'][key] for ep in bottom_3]
        print(f"  {key}: {np.mean(values):.3f} ± {np.std(values):.3f}")
    
    print(f"\nSaved to: {output_file}")

if __name__ == "__main__":
    main()