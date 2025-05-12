#include "can_wrapper.h"
#include <Python.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static PyObject *pModule = NULL, *pBus = NULL;
static int is_python_initialized = 0;
static PyGILState_STATE gil_state;

void acquire_gil(void)
{
    gil_state = PyGILState_Ensure();
}

void release_gil(void)
{
    PyGILState_Release(gil_state);
}

int initialize_python_can(void)
{
    PyObject *pName, *pInitFunc;

    if (is_python_initialized)
    {
        return 1;
    }

    Py_Initialize();

    PyEval_InitThreads();

    // Add current directory to Python path
    PyRun_SimpleString("import sys; sys.path.append('.')");

    pName = PyUnicode_DecodeFSDefault("can_interface");
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule == NULL)
    {
        PyErr_Print();
        fprintf(stderr, "Failed to load Python module 'can_interface'\n");
        return 0;
    }

    pInitFunc = PyObject_GetAttrString(pModule, "initialize_can_bus");
    if (!(pInitFunc && PyCallable_Check(pInitFunc)))
    {
        Py_XDECREF(pInitFunc);
        fprintf(stderr, "Cannot find function 'initialize_can_bus'\n");
        return 0;
    }

    pBus = PyObject_CallObject(pInitFunc, NULL);
    Py_DECREF(pInitFunc);

    if (pBus == NULL)
    {
        PyErr_Print();
        fprintf(stderr, "Failed to initialize CAN bus\n");
        return 0;
    }

    is_python_initialized = 1;

    // Save main thread state and release GIL for other threads
    PyEval_SaveThread();

    return 1;
}

int get_single_bit(const char *function_name)
{
    PyObject *pFunc, *pValue;
    int result = -1;

    // Check initialization
    if (!is_python_initialized || pModule == NULL)
    {
        fprintf(stderr, "Python not initialized. Call initialize_python_can() first.\n");
        return -1;
    }

    acquire_gil();

    pFunc = PyObject_GetAttrString(pModule, function_name);
    if (!(pFunc && PyCallable_Check(pFunc)))
    {
        Py_XDECREF(pFunc);
        fprintf(stderr, "Cannot find function '%s'\n", function_name);
        release_gil();
        return -1;
    }

    PyObject *pArgs = PyTuple_New(1);
    if (pBus == NULL)
    {
        Py_DECREF(pArgs);
        Py_DECREF(pFunc);
        fprintf(stderr, "CAN bus not initialized\n");
        release_gil();
        return -1;
    }

    Py_INCREF(pBus);
    PyTuple_SetItem(pArgs, 0, pBus);

    pValue = PyObject_CallObject(pFunc, pArgs);
    Py_DECREF(pArgs);
    Py_DECREF(pFunc);

    if (pValue == NULL)
    {
        PyErr_Print();
        release_gil();
        return -1;
    }

    if (pValue == Py_None)
    {
        Py_DECREF(pValue);
        release_gil();
        return -1;
    }

    result = PyLong_AsLong(pValue);
    Py_DECREF(pValue);

    release_gil();

    return result;
}

int get_enable_button(void)
{
    return get_single_bit("get_enable");
}

int get_speed_button(void)
{
    return get_single_bit("get_speed");
}

int get_horn_button(void)
{
    return get_single_bit("get_horn");
}

int get_can_enable_button(void)
{
    return get_single_bit("get_can_enable");
}

int get_estop_button(void)
{
    return get_single_bit("get_estop");
}

int get_x_axis(void)
{
    return get_single_bit("get_x_axis");
}

int get_y_axis(void)
{
    return get_single_bit("get_y_axis");
}

int set_led_state(const char *function_name, int state)
{
    PyObject *pFunc, *pResult;

    if (!is_python_initialized || pModule == NULL)
    {
        fprintf(stderr, "Python not initialized. Call initialize_python_can() first.\n");
        return 0;
    }

    acquire_gil();

    pFunc = PyObject_GetAttrString(pModule, function_name);
    if (!(pFunc && PyCallable_Check(pFunc)))
    {
        Py_XDECREF(pFunc);
        fprintf(stderr, "Cannot find function '%s'\n", function_name);
        release_gil();
        return 0;
    }

    PyObject *pArgs = PyTuple_New(2);
    if (pBus == NULL)
    {
        Py_DECREF(pArgs);
        Py_DECREF(pFunc);
        fprintf(stderr, "CAN bus not initialized\n");
        release_gil();
        /
            return 0;
    }

    Py_INCREF(pBus);
    PyTuple_SetItem(pArgs, 0, pBus);
    PyTuple_SetItem(pArgs, 1, PyBool_FromLong(state));

    pResult = PyObject_CallObject(pFunc, pArgs);
    Py_DECREF(pArgs);
    Py_DECREF(pFunc);

    if (pResult == NULL)
    {
        PyErr_Print();
        release_gil();
        return 0;
    }

    Py_DECREF(pResult);

    release_gil();

    return 1;
}

int set_yellow_bat_led(int state)
{
    return set_led_state("set_yellow_bat_led", state);
}

int set_red_bat_led(int state)
{
    return set_led_state("set_red_bat_led", state);
}

int set_overload_led(int state)
{
    return set_led_state("set_overload_led", state);
}

int set_aux_led(int state)
{
    return set_led_state("set_aux_led", state);
}