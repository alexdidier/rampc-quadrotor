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
    QByteArrayData data[20];
    char stringdata0[389];
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
QT_MOC_LITERAL(6, 48, 22), // "on_removeTable_clicked"
QT_MOC_LITERAL(7, 71, 33), // "on_radioButton_table_mode_tog..."
QT_MOC_LITERAL(8, 105, 7), // "checked"
QT_MOC_LITERAL(9, 113, 42), // "on_radioButton_crazyfly_zones..."
QT_MOC_LITERAL(10, 156, 27), // "handleTablePiecesNumChanged"
QT_MOC_LITERAL(11, 184, 6), // "newNum"
QT_MOC_LITERAL(12, 191, 32), // "on_radioButton_lock_mode_toggled"
QT_MOC_LITERAL(13, 224, 24), // "on_checkBox_grid_toggled"
QT_MOC_LITERAL(14, 249, 25), // "on_checkBox_table_toggled"
QT_MOC_LITERAL(15, 275, 34), // "on_checkBox_crazyfly_zones_to..."
QT_MOC_LITERAL(16, 310, 27), // "on_tabWidget_currentChanged"
QT_MOC_LITERAL(17, 338, 5), // "index"
QT_MOC_LITERAL(18, 344, 15), // "centerViewIndex"
QT_MOC_LITERAL(19, 360, 28) // "on_pushButton_fitAll_clicked"

    },
    "MainGUIWindow\0set_tabs\0\0n\0transitionToMode\0"
    "mode\0on_removeTable_clicked\0"
    "on_radioButton_table_mode_toggled\0"
    "checked\0on_radioButton_crazyfly_zones_mode_toggled\0"
    "handleTablePiecesNumChanged\0newNum\0"
    "on_radioButton_lock_mode_toggled\0"
    "on_checkBox_grid_toggled\0"
    "on_checkBox_table_toggled\0"
    "on_checkBox_crazyfly_zones_toggled\0"
    "on_tabWidget_currentChanged\0index\0"
    "centerViewIndex\0on_pushButton_fitAll_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainGUIWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   79,    2, 0x08 /* Private */,
       4,    1,   82,    2, 0x08 /* Private */,
       6,    0,   85,    2, 0x08 /* Private */,
       7,    1,   86,    2, 0x08 /* Private */,
       9,    1,   89,    2, 0x08 /* Private */,
      10,    1,   92,    2, 0x08 /* Private */,
      12,    1,   95,    2, 0x08 /* Private */,
      13,    1,   98,    2, 0x08 /* Private */,
      14,    1,  101,    2, 0x08 /* Private */,
      15,    1,  104,    2, 0x08 /* Private */,
      16,    1,  107,    2, 0x08 /* Private */,
      18,    1,  110,    2, 0x08 /* Private */,
      19,    0,  113,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void, QMetaType::Int,   11,
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
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
        case 2: _t->on_removeTable_clicked(); break;
        case 3: _t->on_radioButton_table_mode_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_radioButton_crazyfly_zones_mode_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->handleTablePiecesNumChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->on_radioButton_lock_mode_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_checkBox_grid_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_checkBox_table_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->on_checkBox_crazyfly_zones_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->on_tabWidget_currentChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->centerViewIndex((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_pushButton_fitAll_clicked(); break;
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
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
