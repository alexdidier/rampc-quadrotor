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
    QByteArrayData data[8];
    char stringdata0[100];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainGUIWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainGUIWindow_t qt_meta_stringdata_MainGUIWindow = {
    {
QT_MOC_LITERAL(0, 0, 13), // "MainGUIWindow"
QT_MOC_LITERAL(1, 14, 8), // "set_tabs"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 1), // "n"
QT_MOC_LITERAL(4, 26, 16), // "transitionToMode"
QT_MOC_LITERAL(5, 43, 4), // "mode"
QT_MOC_LITERAL(6, 48, 28), // "on_drawingModeButton_clicked"
QT_MOC_LITERAL(7, 77, 22) // "on_removeTable_clicked"

    },
    "MainGUIWindow\0set_tabs\0\0n\0transitionToMode\0"
    "mode\0on_drawingModeButton_clicked\0"
    "on_removeTable_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainGUIWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x08 /* Private */,
       4,    1,   37,    2, 0x08 /* Private */,
       6,    0,   40,    2, 0x08 /* Private */,
       7,    0,   41,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainGUIWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainGUIWindow *_t = static_cast<MainGUIWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->set_tabs((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->transitionToMode((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->on_drawingModeButton_clicked(); break;
        case 3: _t->on_removeTable_clicked(); break;
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
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
