/****************************************************************************
** Meta object code from reading C++ file 'myGraphicsScene.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "myGraphicsScene.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'myGraphicsScene.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_myGraphicsScene_t {
    QByteArrayData data[13];
    char stringdata0[180];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_myGraphicsScene_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_myGraphicsScene_t qt_meta_stringdata_myGraphicsScene = {
    {
QT_MOC_LITERAL(0, 0, 15), // "myGraphicsScene"
QT_MOC_LITERAL(1, 16, 23), // "numCrazyFlyZonesChanged"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 6), // "newNum"
QT_MOC_LITERAL(4, 48, 20), // "crazyFlyZoneSelected"
QT_MOC_LITERAL(5, 69, 5), // "index"
QT_MOC_LITERAL(6, 75, 11), // "modeChanged"
QT_MOC_LITERAL(7, 87, 4), // "mode"
QT_MOC_LITERAL(8, 92, 21), // "numTablePiecesChanged"
QT_MOC_LITERAL(9, 114, 18), // "removeCrazyFlyZone"
QT_MOC_LITERAL(10, 133, 23), // "setSelectedCrazyFlyZone"
QT_MOC_LITERAL(11, 157, 12), // "changeModeTo"
QT_MOC_LITERAL(12, 170, 9) // "next_mode"

    },
    "myGraphicsScene\0numCrazyFlyZonesChanged\0"
    "\0newNum\0crazyFlyZoneSelected\0index\0"
    "modeChanged\0mode\0numTablePiecesChanged\0"
    "removeCrazyFlyZone\0setSelectedCrazyFlyZone\0"
    "changeModeTo\0next_mode"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_myGraphicsScene[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,
       4,    1,   52,    2, 0x06 /* Public */,
       6,    1,   55,    2, 0x06 /* Public */,
       8,    1,   58,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    1,   61,    2, 0x0a /* Public */,
      10,    1,   64,    2, 0x0a /* Public */,
      11,    1,   67,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, QMetaType::Int,    3,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,   12,

       0        // eod
};

void myGraphicsScene::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        myGraphicsScene *_t = static_cast<myGraphicsScene *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->numCrazyFlyZonesChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->crazyFlyZoneSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->modeChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->numTablePiecesChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->removeCrazyFlyZone((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->setSelectedCrazyFlyZone((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->changeModeTo((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (myGraphicsScene::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&myGraphicsScene::numCrazyFlyZonesChanged)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (myGraphicsScene::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&myGraphicsScene::crazyFlyZoneSelected)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (myGraphicsScene::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&myGraphicsScene::modeChanged)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (myGraphicsScene::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&myGraphicsScene::numTablePiecesChanged)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject myGraphicsScene::staticMetaObject = {
    { &QGraphicsScene::staticMetaObject, qt_meta_stringdata_myGraphicsScene.data,
      qt_meta_data_myGraphicsScene,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *myGraphicsScene::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *myGraphicsScene::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_myGraphicsScene.stringdata0))
        return static_cast<void*>(const_cast< myGraphicsScene*>(this));
    return QGraphicsScene::qt_metacast(_clname);
}

int myGraphicsScene::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGraphicsScene::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void myGraphicsScene::numCrazyFlyZonesChanged(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void myGraphicsScene::crazyFlyZoneSelected(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void myGraphicsScene::modeChanged(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void myGraphicsScene::numTablePiecesChanged(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
