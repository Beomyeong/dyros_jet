/****************************************************************************
** Meta object code from reading C++ file 'status_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/dyros_jet/dyros_jet_gui/include/dyros_jet_gui/status_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'status_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_dyros_jet_gui__StatusWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      29,   28,   28,   28, 0x0a,
      56,   28,   28,   28, 0x0a,
      81,   28,   28,   28, 0x0a,
     107,   28,   28,   28, 0x0a,
     128,   28,   28,   28, 0x0a,
     146,   28,   28,   28, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_dyros_jet_gui__StatusWindow[] = {
    "dyros_jet_gui::StatusWindow\0\0"
    "on_actionAbout_triggered()\0"
    "on_button_scan_clicked()\0"
    "on_button_estop_clicked()\0"
    "stateButtonClicked()\0updateRecogInfo()\0"
    "updateStateView()\0"
};

void dyros_jet_gui::StatusWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        StatusWindow *_t = static_cast<StatusWindow *>(_o);
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_button_scan_clicked(); break;
        case 2: _t->on_button_estop_clicked(); break;
        case 3: _t->stateButtonClicked(); break;
        case 4: _t->updateRecogInfo(); break;
        case 5: _t->updateStateView(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData dyros_jet_gui::StatusWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject dyros_jet_gui::StatusWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_dyros_jet_gui__StatusWindow,
      qt_meta_data_dyros_jet_gui__StatusWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &dyros_jet_gui::StatusWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *dyros_jet_gui::StatusWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *dyros_jet_gui::StatusWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_dyros_jet_gui__StatusWindow))
        return static_cast<void*>(const_cast< StatusWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int dyros_jet_gui::StatusWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
