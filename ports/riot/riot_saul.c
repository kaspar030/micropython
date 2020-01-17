/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Koen Zandberg "koen@bergzand.net"
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "py/mpconfig.h"
#if MICROPY_PY_RIOT_SAUL

#include "py/runtime.h"
#include "py/mperrno.h"

#include "saul.h"
#include "saul_reg.h"
#include "phydat.h"
#include <math.h>

const mp_obj_type_t riot_saul_reg_type;
const mp_obj_type_t riot_saul_dev_undef_type;
const mp_obj_type_t riot_saul_dev_sense_type;
const mp_obj_type_t riot_saul_dev_act_type;
const mp_obj_base_t riot_saul_dev_undef_obj_template = {&riot_saul_dev_undef_type};
const mp_obj_base_t riot_saul_dev_sense_obj_template = {&riot_saul_dev_sense_type};
const mp_obj_base_t riot_saul_dev_act_obj_template = {&riot_saul_dev_act_type};

/// \moduleref riot
/// \class saul - Bindings for the RIOT Sensor Actuator Uber layer.
///
/// SAUL provides a generic actuator/sensor interface in RIOT. This module
/// provides bindings to interface with SAUL.
///
/// Usage Model:
///
/// First retrieve the SAUL registry object
///
///     reg = saul.get_registry()
///
/// The registry object provides methods to find and retrieve specific SAUL
/// device registrations from the global registry. There is only a single global
/// registry which is always the same object returned with the get call.
///
/// Registered devices can be retrieved in multiple ways. The registry object
/// supports finding devices by index, by type and by name:
///
///      dev_foo = reg[0]
///      dev_bar = reg.find_name("foo")
///      dev_foobar = reg.find_type(saul.ACT_SWITCH)
///
/// Registered devices are implemented as singletons. The same device object is
/// returned when requesting the same index multiple times.
///
/// Iterating over the devices in the registry is also possible:
///
///      for device in reg:
///          print(device.read())
///
/// A device supports reading and writing values, depending on the device type
/// and the driver implementation. A NotImplemented exception will be raised if
/// the underlying device driver does not support the call.
///
/// A successful read from a device will return a list of floating point numbers
/// with the list length depending on the number of dimensions returned by the
/// device.
///
///      temp = thermometer.read()[0]
///
///      y = device_accel.read()[1]
///
/// A write call for a device must be supplied with either a single numerical
/// value or with a list of values in case multiple dimensions of data are
/// supported
///
///      switch.write(1)
///      switch.write([0])
///
/// The type of the device is available in the type property:
///
///      switch.type()
///
/// All type number have a matching property in the saul module:
///
///     switch.type() == saul.ACT_SWITCH

typedef struct _riot_saul_reg_obj_t {
    mp_obj_base_t base;
    saul_reg_t **saul_reg;
    saul_reg_t *iter;
    mp_obj_t list;
} riot_saul_reg_obj_t;

typedef struct _riot_saul_device_obj_t {
    mp_obj_base_t base;
    saul_reg_t *saul_reg;
} riot_saul_dev_obj_t;

STATIC riot_saul_reg_obj_t riot_saul_reg_obj = {{&riot_saul_reg_type},
                                                .saul_reg = &saul_reg,
                                                .list = MP_OBJ_NULL};

STATIC void riot_saul_dev_print(const mp_print_t *print,
                                mp_obj_t self_in, mp_print_kind_t kind) {
    riot_saul_dev_obj_t *self = self_in;
    mp_printf(print, "SAUL device: \"%s\"", self->saul_reg->name);
}

STATIC void riot_saul_dev_sense_print(const mp_print_t *print,
                                mp_obj_t self_in, mp_print_kind_t kind) {
    riot_saul_dev_obj_t *self = self_in;
    mp_printf(print, "SAUL sensor: \"%s\"", self->saul_reg->name);
}

STATIC void riot_saul_dev_act_print(const mp_print_t *print,
                                mp_obj_t self_in, mp_print_kind_t kind) {
    riot_saul_dev_obj_t *self = self_in;
    mp_printf(print, "SAUL actuator: \"%s\"", self->saul_reg->name);
}

static float _phydat_to_float(int16_t val, int8_t scale) {
    float fval = (float)val;
    return fval * powf(10, scale);
}

STATIC mp_obj_t riot_saul_dev_type(mp_obj_t self_in) {
    riot_saul_dev_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return MP_OBJ_NEW_SMALL_INT(self->saul_reg->driver->type);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(riot_saul_dev_type_obj,
                                 riot_saul_dev_type);

STATIC mp_obj_t riot_saul_dev_read(mp_obj_t self_in) {
    phydat_t data = {.val = {PHYDAT_MIN, PHYDAT_MIN, PHYDAT_MIN}};
    riot_saul_dev_obj_t *self = self_in;

    int res = saul_reg_read(self->saul_reg, &data);

    if (res == -ENODEV) {
        mp_raise_OSError(MP_ENODEV);
    }
    else if (res == -ENOTSUP) {
        mp_raise_NotImplementedError("Read not supported by device");
    }
    else if (res == -ECANCELED) {
        mp_raise_OSError(MP_EIO);
    }
    mp_obj_t data_list = mp_obj_new_list(0, NULL);
    for (size_t i = 0; i < (size_t)res; i++) {
        float fdata = _phydat_to_float(data.val[i], data.scale);
        mp_obj_list_append(data_list, mp_obj_new_float(fdata));
    }
    return MP_OBJ_TO_PTR(data_list);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(riot_saul_dev_read_obj, riot_saul_dev_read);

STATIC mp_obj_t riot_saul_dev_write(mp_obj_t self_in, mp_obj_t values) {
    riot_saul_dev_obj_t *self = self_in;
    phydat_t data;
    int32_t ilist[PHYDAT_DIM] = { 0 };
    size_t len = 1;
    if (mp_obj_is_int(values)) {
        ilist[0] = mp_obj_get_int(values);
    }
    else {
        mp_obj_t *items;
        mp_obj_get_array(values, &len, &items);
        if (len > PHYDAT_DIM) {
            mp_raise_ValueError("Must supply max PHYDAT_DIM values");
        }
        for (size_t i = 0; i < len; i++) {
            ilist[i] = (int32_t)mp_obj_get_int(items[i]);
        }
    }
    phydat_fit(&data, ilist, len);
    int res = saul_reg_write(self->saul_reg, &data);
    if (res == -ENODEV) {
        mp_raise_OSError(MP_ENODEV);
    }
    else if (res == -ENOTSUP) {
        mp_raise_NotImplementedError("Write not supported by device");
    }
    else if (res == -ECANCELED) {
        mp_raise_OSError(MP_EIO);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(riot_saul_dev_write_obj, riot_saul_dev_write);

// SAUL device methods and properties
STATIC const mp_rom_map_elem_t riot_saul_dev_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_read),  MP_ROM_PTR(&riot_saul_dev_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),  MP_ROM_PTR(&riot_saul_dev_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_type),  MP_ROM_PTR(&riot_saul_dev_type_obj) },
};

STATIC MP_DEFINE_CONST_DICT(riot_saul_dev_locals_dict,
                            riot_saul_dev_locals_dict_table);

const mp_obj_type_t riot_saul_dev_undef_type = {
    { &mp_type_type },
    .name = MP_QSTR_saul_device,
    .print = riot_saul_dev_print,
    .locals_dict = (mp_obj_t)&riot_saul_dev_locals_dict,
};

const mp_obj_type_t riot_saul_dev_act_type = {
    { &mp_type_type },
    .name = MP_QSTR_saul_actuator,
    .print = riot_saul_dev_act_print,
    .locals_dict = (mp_obj_t)&riot_saul_dev_locals_dict,
    .parent = &riot_saul_dev_undef_type,
};

const mp_obj_type_t riot_saul_dev_sense_type = {
    { &mp_type_type },
    .name = MP_QSTR_saul_sensor,
    .print = riot_saul_dev_sense_print,
    .locals_dict = (mp_obj_t)&riot_saul_dev_locals_dict,
    .parent = &riot_saul_dev_undef_type,
};

// SAUL registry methods and properties
STATIC void riot_saul_reg_print(const mp_print_t *print,
                                mp_obj_t self_in, mp_print_kind_t kind) {
    riot_saul_reg_obj_t *self = self_in;
    mp_printf(print, "SAUL(%p)", self->saul_reg);
}

STATIC mp_obj_t _get_mp_saul_device(riot_saul_reg_obj_t *self, saul_reg_t *device)
{
    mp_obj_list_t *list = MP_OBJ_TO_PTR(self->list);
    mp_obj_t *devices;
    size_t len = 0;

    mp_obj_list_get(list, &len, &devices);
    for (size_t i = 0; i < len; i++) {
        riot_saul_dev_obj_t *dev_obj = MP_OBJ_TO_PTR(devices[i]);
        if (dev_obj->saul_reg == device) {
            return devices[i];
        }
    }

    // Device not found, instantiate singleton
    riot_saul_dev_obj_t *dev_obj = m_new_obj(riot_saul_dev_obj_t);

    uint8_t category = device->driver->type & SAUL_CAT_MASK;

    if (category ==  SAUL_CAT_ACT) {
        dev_obj->base = riot_saul_dev_act_obj_template;
    }
    else if (category == SAUL_CAT_SENSE) {
        dev_obj->base = riot_saul_dev_sense_obj_template;
    }
    else {
        dev_obj->base = riot_saul_dev_undef_obj_template;
    }
    dev_obj->saul_reg = device;

    mp_obj_list_append(list, dev_obj);
    return dev_obj;
}

STATIC mp_obj_t riot_saul_reg_find_type(mp_obj_t self_in, mp_obj_t type) {
    riot_saul_reg_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int itype = mp_obj_get_int(type);

    if (itype < 0 || itype > SAUL_CLASS_ANY) {
        mp_raise_msg(&mp_type_ValueError, "Type out of bounds");
    }

    saul_reg_t *device = saul_reg_find_type((uint8_t)itype);
    if (device == NULL) {
        mp_raise_msg(&mp_type_IndexError, "No SAUL device registry of type");
    }

    mp_obj_t dev_obj = _get_mp_saul_device(self, device);

    return MP_OBJ_TO_PTR(dev_obj);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(riot_saul_reg_find_type_obj,
                                 riot_saul_reg_find_type);

STATIC mp_obj_t riot_saul_reg_find_name(mp_obj_t self_in, mp_obj_t name) {
    riot_saul_reg_obj_t *self = MP_OBJ_TO_PTR(self_in);
    const char *sname = mp_obj_str_get_str(name);

    saul_reg_t *device = saul_reg_find_name(sname);
    if (device == NULL) {
        mp_raise_msg(&mp_type_IndexError, "No SAUL device with that type");
    }

    mp_obj_t dev_obj = _get_mp_saul_device(self, device);

    return MP_OBJ_TO_PTR(dev_obj);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(riot_saul_reg_find_name_obj,
                                 riot_saul_reg_find_name);

STATIC mp_obj_t riot_saul_reg_iter(mp_obj_t self_in) {
    riot_saul_reg_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->iter == NULL) {
        self->iter = *self->saul_reg;
        return MP_OBJ_STOP_ITERATION;
    }
    mp_obj_t dev_obj = _get_mp_saul_device(self, self->iter);
    self->iter = self->iter->next;
    return MP_OBJ_TO_PTR(dev_obj);
}

STATIC mp_obj_t riot_saul_reg_subscr(mp_obj_t self_in, mp_obj_t index, mp_obj_t value) {
    riot_saul_reg_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (value == MP_OBJ_SENTINEL) {
        int sidx = mp_obj_get_int(index);
        if (sidx < 0) {
            mp_raise_msg(&mp_type_ValueError, "Negative index not supported");
        }
        size_t idx = (size_t)sidx;

        saul_reg_t *device = saul_reg_find_nth(idx);
        if (device == NULL) {
            mp_raise_msg(&mp_type_IndexError, "No SAUL device with that name");
        }

        mp_obj_t dev_obj = _get_mp_saul_device(self, device);

        return MP_OBJ_TO_PTR(dev_obj);
    } else {
        return MP_OBJ_NULL; // op not supported
    }
}

STATIC const mp_rom_map_elem_t riot_saul_reg_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_find_type), MP_ROM_PTR(&riot_saul_reg_find_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_find_name), MP_ROM_PTR(&riot_saul_reg_find_name_obj) },
};

STATIC MP_DEFINE_CONST_DICT(riot_saul_reg_locals_dict,
                            riot_saul_reg_locals_dict_table);

const mp_obj_type_t riot_saul_reg_type = {
    { &mp_type_type },
    .name = MP_QSTR_saul_registry,
    .print = riot_saul_reg_print,
    .getiter = mp_identity_getiter,
    .iternext = riot_saul_reg_iter,
    .subscr = riot_saul_reg_subscr,
    .locals_dict = (mp_obj_t)&riot_saul_reg_locals_dict,
};

STATIC mp_obj_t mod_get_registry(void) {
    if (riot_saul_reg_obj.list == MP_OBJ_NULL) {
        riot_saul_reg_obj.list = mp_obj_new_list(0, NULL);
        riot_saul_reg_obj.iter = *riot_saul_reg_obj.saul_reg;
    }
    return MP_OBJ_FROM_PTR(&riot_saul_reg_obj);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_get_registry_obj, mod_get_registry);

// SAUL module methods and properties
STATIC const mp_rom_map_elem_t mp_module_saul_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),            MP_ROM_QSTR(MP_QSTR_saul) },
    { MP_ROM_QSTR(MP_QSTR_get_registry),        MP_ROM_PTR(&mod_get_registry_obj) },
    { MP_ROM_QSTR(MP_QSTR_act),                 MP_ROM_PTR(&riot_saul_dev_act_type) },
    { MP_ROM_QSTR(MP_QSTR_sense),               MP_ROM_PTR(&riot_saul_dev_sense_type) },
    { MP_ROM_QSTR(MP_QSTR_undef),               MP_ROM_PTR(&riot_saul_dev_undef_type) },

    // SAUL category types
    { MP_ROM_QSTR(MP_QSTR_ACT_ANY),            MP_ROM_INT(SAUL_ACT_ANY) },
    { MP_ROM_QSTR(MP_QSTR_ACT_LED_RGB),        MP_ROM_INT(SAUL_ACT_LED_RGB) },
    { MP_ROM_QSTR(MP_QSTR_ACT_SERVO),          MP_ROM_INT(SAUL_ACT_SERVO) },
    { MP_ROM_QSTR(MP_QSTR_ACT_MOTOR),          MP_ROM_INT(SAUL_ACT_MOTOR) },
    { MP_ROM_QSTR(MP_QSTR_ACT_SWITCH),         MP_ROM_INT(SAUL_ACT_SWITCH) },
    { MP_ROM_QSTR(MP_QSTR_ACT_DIMMER),         MP_ROM_INT(SAUL_ACT_DIMMER) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_ANY),          MP_ROM_INT(SAUL_SENSE_ANY) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_BTN),          MP_ROM_INT(SAUL_SENSE_BTN) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_TEMP),         MP_ROM_INT(SAUL_SENSE_TEMP) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_HUM),          MP_ROM_INT(SAUL_SENSE_HUM) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_LIGHT),        MP_ROM_INT(SAUL_SENSE_LIGHT) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_ACCEL),        MP_ROM_INT(SAUL_SENSE_ACCEL) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_MAG),          MP_ROM_INT(SAUL_SENSE_MAG) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_GYRO),         MP_ROM_INT(SAUL_SENSE_GYRO) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_COLOR),        MP_ROM_INT(SAUL_SENSE_COLOR) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_PRESS),        MP_ROM_INT(SAUL_SENSE_PRESS) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_ANALOG),       MP_ROM_INT(SAUL_SENSE_ANALOG) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_UV),           MP_ROM_INT(SAUL_SENSE_UV) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_OBJTEMP),      MP_ROM_INT(SAUL_SENSE_OBJTEMP) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_COUNT),        MP_ROM_INT(SAUL_SENSE_COUNT) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_DISTANCE),     MP_ROM_INT(SAUL_SENSE_DISTANCE) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_CO2),          MP_ROM_INT(SAUL_SENSE_CO2) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_TVOC),         MP_ROM_INT(SAUL_SENSE_TVOC) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_OCCUP),        MP_ROM_INT(SAUL_SENSE_OCCUP) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_PROXIMITY),    MP_ROM_INT(SAUL_SENSE_PROXIMITY) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_RSSI),         MP_ROM_INT(SAUL_SENSE_RSSI) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_CHARGE),       MP_ROM_INT(SAUL_SENSE_CHARGE) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_CURRENT),      MP_ROM_INT(SAUL_SENSE_CURRENT) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_PM),           MP_ROM_INT(SAUL_SENSE_PM) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_CAPACITANCE),  MP_ROM_INT(SAUL_SENSE_CAPACITANCE) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_VOLTAGE),      MP_ROM_INT(SAUL_SENSE_VOLTAGE) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_PH),           MP_ROM_INT(SAUL_SENSE_PH) },
    { MP_ROM_QSTR(MP_QSTR_SENSE_POWER),        MP_ROM_INT(SAUL_SENSE_POWER) },
    { MP_ROM_QSTR(MP_QSTR_CLASS_ANY),          MP_ROM_INT(SAUL_CLASS_ANY) },
};

STATIC MP_DEFINE_CONST_DICT(mp_module_saul_globals, mp_module_saul_globals_table);

const mp_obj_module_t mp_module_riot_saul = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_saul_globals,
};
#endif // MICROPY_PY_RIOT_SAUL
