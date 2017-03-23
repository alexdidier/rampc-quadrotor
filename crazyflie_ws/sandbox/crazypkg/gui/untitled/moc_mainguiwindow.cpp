/****************************************************************************
** Meta object code from reading C++ file 'mainguiwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainguiwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainguiwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainGUIWindow_t {
    QByteArrayData data[9];
    char stringdata0[172];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainGUIWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainGUIWindow_t qt_meta_stringdata_MainGUIWindow = {
    {
QT_MOC_LITERAL(0, 0, 13), // "MainGUIWindow"
QT_MOC_LITERAL(1, 14, 36), // "on_spinBoxNumCrazyflies_value..."
QT_MOC_LITERAL(2, 51, 0), // ""
QT_MOC_LITERAL(3, 52, 4), // "arg1"
QT_MOC_LITERAL(4, 57, 39), // "on_spinBoxNumCrazyflies_editi..."
QT_MOC_LITERAL(5, 97, 33), // "on_graphicsView_rubberBandCha..."
QT_MOC_LITERAL(6, 131, 12), // "viewportRect"
QT_MOC_LITERAL(7, 144, 14), // "fromScenePoint"
QT_MOC_LITERAL(8, 159, 12) // "toScenePoint"

    },
    "MainGUIWindow\0on_spinBoxNumCrazyflies_valueChanged\0"
    "\0arg1\0on_spinBoxNumCrazyflies_editingFinished\0"
    "on_graphicsView_rubberBandChanged\0"
    "viewportRect\0fromScenePoint\0toScenePoint"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainGUIWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x08 /* Private */,
       4,    0,   32,    2, 0x08 /* Private */,
       5,    3,   33,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QRect, QMetaType::QPointF, QMetaType::QPointF,    6,    7,    8,

       0        // eod
};

void MainGUIWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainGUIWindow *_t = static_cast<MainGUIWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_spinBoxNumCrazyflies_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->on_spinBoxNumCrazyflies_editingFinished(); break;
        case 2: _t->on_graphicsView_rubberBandChanged((*reinterpret_cast< const QRect(*)>(_a[1])),(*reinterpret_cast< const QPointF(*)>(_a[2])),(*reinterpret_cast< const QPointF(*)>(_a[3]))); break;
        default: ;
        }
    }
}

const QMetaObject MainGUIWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainGUIWindow.data,
      qt_meta_data_MainGUIWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainGUIWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainGUIWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainGUIWindow.stringdata0))
        return static_cast<void*>(const_cast< MainGUIWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainGUIWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
