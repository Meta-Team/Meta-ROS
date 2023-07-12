// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from can_interfaces:msg/PosVelData.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "can_interfaces/msg/detail/pos_vel_data__struct.h"
#include "can_interfaces/msg/detail/pos_vel_data__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool can_interfaces__msg__pos_vel_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("can_interfaces.msg._pos_vel_data.PosVelData", full_classname_dest, 43) == 0);
  }
  can_interfaces__msg__PosVelData * ros_message = _ros_message;
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // p_des
    PyObject * field = PyObject_GetAttrString(_pymsg, "p_des");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->p_des = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // v_des
    PyObject * field = PyObject_GetAttrString(_pymsg, "v_des");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->v_des = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * can_interfaces__msg__pos_vel_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PosVelData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("can_interfaces.msg._pos_vel_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PosVelData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  can_interfaces__msg__PosVelData * ros_message = (can_interfaces__msg__PosVelData *)raw_ros_message;
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // p_des
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->p_des);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p_des", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // v_des
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->v_des);
    {
      int rc = PyObject_SetAttrString(_pymessage, "v_des", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
