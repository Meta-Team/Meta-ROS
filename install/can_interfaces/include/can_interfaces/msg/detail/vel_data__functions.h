// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from can_interfaces:msg/VelData.idl
// generated code does not contain a copyright notice

#ifndef CAN_INTERFACES__MSG__DETAIL__VEL_DATA__FUNCTIONS_H_
#define CAN_INTERFACES__MSG__DETAIL__VEL_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "can_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "can_interfaces/msg/detail/vel_data__struct.h"

/// Initialize msg/VelData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * can_interfaces__msg__VelData
 * )) before or use
 * can_interfaces__msg__VelData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
bool
can_interfaces__msg__VelData__init(can_interfaces__msg__VelData * msg);

/// Finalize msg/VelData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
void
can_interfaces__msg__VelData__fini(can_interfaces__msg__VelData * msg);

/// Create msg/VelData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * can_interfaces__msg__VelData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
can_interfaces__msg__VelData *
can_interfaces__msg__VelData__create();

/// Destroy msg/VelData message.
/**
 * It calls
 * can_interfaces__msg__VelData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
void
can_interfaces__msg__VelData__destroy(can_interfaces__msg__VelData * msg);

/// Check for msg/VelData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
bool
can_interfaces__msg__VelData__are_equal(const can_interfaces__msg__VelData * lhs, const can_interfaces__msg__VelData * rhs);

/// Copy a msg/VelData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
bool
can_interfaces__msg__VelData__copy(
  const can_interfaces__msg__VelData * input,
  can_interfaces__msg__VelData * output);

/// Initialize array of msg/VelData messages.
/**
 * It allocates the memory for the number of elements and calls
 * can_interfaces__msg__VelData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
bool
can_interfaces__msg__VelData__Sequence__init(can_interfaces__msg__VelData__Sequence * array, size_t size);

/// Finalize array of msg/VelData messages.
/**
 * It calls
 * can_interfaces__msg__VelData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
void
can_interfaces__msg__VelData__Sequence__fini(can_interfaces__msg__VelData__Sequence * array);

/// Create array of msg/VelData messages.
/**
 * It allocates the memory for the array and calls
 * can_interfaces__msg__VelData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
can_interfaces__msg__VelData__Sequence *
can_interfaces__msg__VelData__Sequence__create(size_t size);

/// Destroy array of msg/VelData messages.
/**
 * It calls
 * can_interfaces__msg__VelData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
void
can_interfaces__msg__VelData__Sequence__destroy(can_interfaces__msg__VelData__Sequence * array);

/// Check for msg/VelData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
bool
can_interfaces__msg__VelData__Sequence__are_equal(const can_interfaces__msg__VelData__Sequence * lhs, const can_interfaces__msg__VelData__Sequence * rhs);

/// Copy an array of msg/VelData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_can_interfaces
bool
can_interfaces__msg__VelData__Sequence__copy(
  const can_interfaces__msg__VelData__Sequence * input,
  can_interfaces__msg__VelData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CAN_INTERFACES__MSG__DETAIL__VEL_DATA__FUNCTIONS_H_
