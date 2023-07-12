// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from can_interfaces:msg/Feedback.idl
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
#include "can_interfaces/msg/detail/feedback__struct.h"
#include "can_interfaces/msg/detail/feedback__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool can_interfaces__msg__feedback__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[38];
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
    assert(strncmp("can_interfaces.msg._feedback.Feedback", full_classname_dest, 37) == 0);
  }
  can_interfaces__msg__Feedback * ros_message = _ros_message;
  {  // mst_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "mst_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mst_id = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // pos
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pos = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tor
    PyObject * field = PyObject_GetAttrString(_pymsg, "tor");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tor = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // t_mos
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_mos");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->t_mos = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // t_rotor
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_rotor");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->t_rotor = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * can_interfaces__msg__feedback__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Feedback */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("can_interfaces.msg._feedback");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Feedback");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  can_interfaces__msg__Feedback * ros_message = (can_interfaces__msg__Feedback *)raw_ros_message;
  {  // mst_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->mst_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mst_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pos);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tor
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t_mos
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->t_mos);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_mos", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t_rotor
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->t_rotor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_rotor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
