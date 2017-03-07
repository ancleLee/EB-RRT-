/****************************************************************************
** Meta object code from reading C++ file 'RRTWidget.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.2.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../nbrrts/RRTWidget.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'RRTWidget.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.2.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_RRTWidget_t {
    QByteArrayData data[22];
    char stringdata[266];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    offsetof(qt_meta_stringdata_RRTWidget_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData) \
    )
static const qt_meta_stringdata_RRTWidget_t qt_meta_stringdata_RRTWidget = {
    {
QT_MOC_LITERAL(0, 0, 9),
QT_MOC_LITERAL(1, 10, 14),
QT_MOC_LITERAL(2, 25, 0),
QT_MOC_LITERAL(3, 26, 14),
QT_MOC_LITERAL(4, 41, 15),
QT_MOC_LITERAL(5, 57, 14),
QT_MOC_LITERAL(6, 72, 8),
QT_MOC_LITERAL(7, 81, 8),
QT_MOC_LITERAL(8, 90, 9),
QT_MOC_LITERAL(9, 100, 10),
QT_MOC_LITERAL(10, 111, 19),
QT_MOC_LITERAL(11, 131, 9),
QT_MOC_LITERAL(12, 141, 12),
QT_MOC_LITERAL(13, 154, 16),
QT_MOC_LITERAL(14, 171, 4),
QT_MOC_LITERAL(15, 176, 20),
QT_MOC_LITERAL(16, 197, 11),
QT_MOC_LITERAL(17, 209, 7),
QT_MOC_LITERAL(18, 217, 16),
QT_MOC_LITERAL(19, 234, 4),
QT_MOC_LITERAL(20, 239, 12),
QT_MOC_LITERAL(21, 252, 12)
    },
    "RRTWidget\0signal_stepped\0\0iterationCount\0"
    "signal_solution\0solutionlength\0slot_run\0"
    "run_step\0slot_stop\0slot_reset\0"
    "slot_clearObstacles\0slot_step\0"
    "slot_stepBig\0slot_setGoalBias\0bias\0"
    "slot_setWaypointBias\0slot_setASC\0"
    "checked\0slot_setStepSize\0step\0"
    "slot_openmap\0slot_savemap\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RRTWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   89,    2, 0x06,
       4,    1,   92,    2, 0x06,

 // slots: name, argc, parameters, tag, flags
       6,    0,   95,    2, 0x08,
       7,    0,   96,    2, 0x08,
       8,    0,   97,    2, 0x08,
       9,    0,   98,    2, 0x08,
      10,    0,   99,    2, 0x08,
      11,    0,  100,    2, 0x08,
      12,    0,  101,    2, 0x08,
      13,    1,  102,    2, 0x08,
      15,    1,  105,    2, 0x08,
      16,    1,  108,    2, 0x08,
      18,    1,  111,    2, 0x08,
      20,    0,  114,    2, 0x08,
      21,    0,  115,    2, 0x08,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Double,    5,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   14,
    QMetaType::Void, QMetaType::Int,   14,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Double,   19,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RRTWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RRTWidget *_t = static_cast<RRTWidget *>(_o);
        switch (_id) {
        case 0: _t->signal_stepped((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->signal_solution((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 2: _t->slot_run(); break;
        case 3: _t->run_step(); break;
        case 4: _t->slot_stop(); break;
        case 5: _t->slot_reset(); break;
        case 6: _t->slot_clearObstacles(); break;
        case 7: _t->slot_step(); break;
        case 8: _t->slot_stepBig(); break;
        case 9: _t->slot_setGoalBias((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->slot_setWaypointBias((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->slot_setASC((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->slot_setStepSize((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 13: _t->slot_openmap(); break;
        case 14: _t->slot_savemap(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (RRTWidget::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RRTWidget::signal_stepped)) {
                *result = 0;
            }
        }
        {
            typedef void (RRTWidget::*_t)(double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RRTWidget::signal_solution)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject RRTWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_RRTWidget.data,
      qt_meta_data_RRTWidget,  qt_static_metacall, 0, 0}
};


const QMetaObject *RRTWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RRTWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_RRTWidget.stringdata))
        return static_cast<void*>(const_cast< RRTWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int RRTWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void RRTWidget::signal_stepped(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void RRTWidget::signal_solution(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
