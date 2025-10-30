// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_interfaces:srv/GetDataFromDxl.idl
// generated code does not contain a copyright notice
#include "dynamixel_interfaces/srv/detail/get_data_from_dxl__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `item_name`
#include "rosidl_runtime_c/string_functions.h"

bool
dynamixel_interfaces__srv__GetDataFromDxl_Request__init(dynamixel_interfaces__srv__GetDataFromDxl_Request * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    dynamixel_interfaces__srv__GetDataFromDxl_Request__fini(msg);
    return false;
  }
  // id
  // item_name
  if (!rosidl_runtime_c__String__init(&msg->item_name)) {
    dynamixel_interfaces__srv__GetDataFromDxl_Request__fini(msg);
    return false;
  }
  // timeout_sec
  return true;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Request__fini(dynamixel_interfaces__srv__GetDataFromDxl_Request * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // item_name
  rosidl_runtime_c__String__fini(&msg->item_name);
  // timeout_sec
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Request__are_equal(const dynamixel_interfaces__srv__GetDataFromDxl_Request * lhs, const dynamixel_interfaces__srv__GetDataFromDxl_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // item_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->item_name), &(rhs->item_name)))
  {
    return false;
  }
  // timeout_sec
  if (lhs->timeout_sec != rhs->timeout_sec) {
    return false;
  }
  return true;
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Request__copy(
  const dynamixel_interfaces__srv__GetDataFromDxl_Request * input,
  dynamixel_interfaces__srv__GetDataFromDxl_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // id
  output->id = input->id;
  // item_name
  if (!rosidl_runtime_c__String__copy(
      &(input->item_name), &(output->item_name)))
  {
    return false;
  }
  // timeout_sec
  output->timeout_sec = input->timeout_sec;
  return true;
}

dynamixel_interfaces__srv__GetDataFromDxl_Request *
dynamixel_interfaces__srv__GetDataFromDxl_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_interfaces__srv__GetDataFromDxl_Request * msg = (dynamixel_interfaces__srv__GetDataFromDxl_Request *)allocator.allocate(sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Request));
  bool success = dynamixel_interfaces__srv__GetDataFromDxl_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Request__destroy(dynamixel_interfaces__srv__GetDataFromDxl_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_interfaces__srv__GetDataFromDxl_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__init(dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_interfaces__srv__GetDataFromDxl_Request * data = NULL;

  if (size) {
    data = (dynamixel_interfaces__srv__GetDataFromDxl_Request *)allocator.zero_allocate(size, sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_interfaces__srv__GetDataFromDxl_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_interfaces__srv__GetDataFromDxl_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__fini(dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dynamixel_interfaces__srv__GetDataFromDxl_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence *
dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence * array = (dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence *)allocator.allocate(sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__destroy(dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__are_equal(const dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence * lhs, const dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_interfaces__srv__GetDataFromDxl_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__copy(
  const dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence * input,
  dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_interfaces__srv__GetDataFromDxl_Request * data =
      (dynamixel_interfaces__srv__GetDataFromDxl_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_interfaces__srv__GetDataFromDxl_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_interfaces__srv__GetDataFromDxl_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_interfaces__srv__GetDataFromDxl_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
dynamixel_interfaces__srv__GetDataFromDxl_Response__init(dynamixel_interfaces__srv__GetDataFromDxl_Response * msg)
{
  if (!msg) {
    return false;
  }
  // item_data
  // result
  return true;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Response__fini(dynamixel_interfaces__srv__GetDataFromDxl_Response * msg)
{
  if (!msg) {
    return;
  }
  // item_data
  // result
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Response__are_equal(const dynamixel_interfaces__srv__GetDataFromDxl_Response * lhs, const dynamixel_interfaces__srv__GetDataFromDxl_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // item_data
  if (lhs->item_data != rhs->item_data) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  return true;
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Response__copy(
  const dynamixel_interfaces__srv__GetDataFromDxl_Response * input,
  dynamixel_interfaces__srv__GetDataFromDxl_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // item_data
  output->item_data = input->item_data;
  // result
  output->result = input->result;
  return true;
}

dynamixel_interfaces__srv__GetDataFromDxl_Response *
dynamixel_interfaces__srv__GetDataFromDxl_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_interfaces__srv__GetDataFromDxl_Response * msg = (dynamixel_interfaces__srv__GetDataFromDxl_Response *)allocator.allocate(sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Response));
  bool success = dynamixel_interfaces__srv__GetDataFromDxl_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Response__destroy(dynamixel_interfaces__srv__GetDataFromDxl_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_interfaces__srv__GetDataFromDxl_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__init(dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_interfaces__srv__GetDataFromDxl_Response * data = NULL;

  if (size) {
    data = (dynamixel_interfaces__srv__GetDataFromDxl_Response *)allocator.zero_allocate(size, sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_interfaces__srv__GetDataFromDxl_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_interfaces__srv__GetDataFromDxl_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__fini(dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dynamixel_interfaces__srv__GetDataFromDxl_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence *
dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence * array = (dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence *)allocator.allocate(sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__destroy(dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__are_equal(const dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence * lhs, const dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_interfaces__srv__GetDataFromDxl_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__copy(
  const dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence * input,
  dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_interfaces__srv__GetDataFromDxl_Response * data =
      (dynamixel_interfaces__srv__GetDataFromDxl_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_interfaces__srv__GetDataFromDxl_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_interfaces__srv__GetDataFromDxl_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_interfaces__srv__GetDataFromDxl_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "dynamixel_interfaces/srv/detail/get_data_from_dxl__functions.h"

bool
dynamixel_interfaces__srv__GetDataFromDxl_Event__init(dynamixel_interfaces__srv__GetDataFromDxl_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    dynamixel_interfaces__srv__GetDataFromDxl_Event__fini(msg);
    return false;
  }
  // request
  if (!dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__init(&msg->request, 0)) {
    dynamixel_interfaces__srv__GetDataFromDxl_Event__fini(msg);
    return false;
  }
  // response
  if (!dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__init(&msg->response, 0)) {
    dynamixel_interfaces__srv__GetDataFromDxl_Event__fini(msg);
    return false;
  }
  return true;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Event__fini(dynamixel_interfaces__srv__GetDataFromDxl_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__fini(&msg->request);
  // response
  dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__fini(&msg->response);
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Event__are_equal(const dynamixel_interfaces__srv__GetDataFromDxl_Event * lhs, const dynamixel_interfaces__srv__GetDataFromDxl_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Event__copy(
  const dynamixel_interfaces__srv__GetDataFromDxl_Event * input,
  dynamixel_interfaces__srv__GetDataFromDxl_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!dynamixel_interfaces__srv__GetDataFromDxl_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!dynamixel_interfaces__srv__GetDataFromDxl_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

dynamixel_interfaces__srv__GetDataFromDxl_Event *
dynamixel_interfaces__srv__GetDataFromDxl_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_interfaces__srv__GetDataFromDxl_Event * msg = (dynamixel_interfaces__srv__GetDataFromDxl_Event *)allocator.allocate(sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Event));
  bool success = dynamixel_interfaces__srv__GetDataFromDxl_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Event__destroy(dynamixel_interfaces__srv__GetDataFromDxl_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_interfaces__srv__GetDataFromDxl_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence__init(dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_interfaces__srv__GetDataFromDxl_Event * data = NULL;

  if (size) {
    data = (dynamixel_interfaces__srv__GetDataFromDxl_Event *)allocator.zero_allocate(size, sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_interfaces__srv__GetDataFromDxl_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_interfaces__srv__GetDataFromDxl_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence__fini(dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dynamixel_interfaces__srv__GetDataFromDxl_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence *
dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence * array = (dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence *)allocator.allocate(sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence__destroy(dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence__are_equal(const dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence * lhs, const dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_interfaces__srv__GetDataFromDxl_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence__copy(
  const dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence * input,
  dynamixel_interfaces__srv__GetDataFromDxl_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_interfaces__srv__GetDataFromDxl_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_interfaces__srv__GetDataFromDxl_Event * data =
      (dynamixel_interfaces__srv__GetDataFromDxl_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_interfaces__srv__GetDataFromDxl_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_interfaces__srv__GetDataFromDxl_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_interfaces__srv__GetDataFromDxl_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
