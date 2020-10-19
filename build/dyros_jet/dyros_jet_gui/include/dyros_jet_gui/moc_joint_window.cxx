/****************************************************************************
** Meta object code from reading C++ file 'joint_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/dyros_jet/dyros_jet_gui/include/dyros_jet_gui/joint_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'joint_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_dyros_jet_gui__JointWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      28,   27,   27,   27, 0x0a,
      55,   27,   27,   27, 0x0a,
      79,   27,   27,   27, 0x0a,
     102,   27,   27,   27, 0x0a,
     124,   27,   27,   27, 0x0a,
     159,  142,   27,   27, 0x0a,
     195,   27,   27,   27, 0x0a,
     226,   27,   27,   27, 0x0a,
     257,   27,   27,   27, 0x0a,
     276,   27,   27,   27, 0x0a,
     302,   27,   27,   27, 0x0a,
     328,   27,   27,   27, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_dyros_jet_gui__JointWindow[] = {
    "dyros_jet_gui::JointWindow\0\0"
    "on_actionAbout_triggered()\0"
    "jointCtrlMinusClicked()\0jointCtrlPlusClicked()\0"
    "jointCtrlSetClicked()\0updateJointView()\0"
    "i,column,newItem\0setTable(int,int,QTableWidgetItem*)\0"
    "on_button_motor_save_clicked()\0"
    "on_button_motor_load_clicked()\0"
    "ConfineChecklist()\0on_all_checkBox_clicked()\0"
    "on_basic_button_clicked()\0"
    "on_detail_button_clicked()\0"
};

void dyros_jet_gui::JointWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        JointWindow *_t = static_cast<JointWindow *>(_o);
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->jointCtrlMinusClicked(); break;
        case 2: _t->jointCtrlPlusClicked(); break;
        case 3: _t->jointCtrlSetClicked(); break;
        case 4: _t->updateJointView(); break;
        case 5: _t->setTable((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< QTableWidgetItem*(*)>(_a[3]))); break;
        case 6: _t->on_button_motor_save_clicked(); break;
        case 7: _t->on_button_motor_load_clicked(); break;
        case 8: _t->ConfineChecklist(); break;
        case 9: _t->on_all_checkBox_clicked(); break;
        case 10: _t->on_basic_button_clicked(); break;
        case 11: _t->on_detail_button_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData dyros_jet_gui::JointWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject dyros_jet_gui::JointWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_dyros_jet_gui__JointWindow,
      qt_meta_data_dyros_jet_gui__JointWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &dyros_jet_gui::JointWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *dyros_jet_gui::JointWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *dyros_jet_gui::JointWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_dyros_jet_gui__JointWindow))
        return static_cast<void*>(const_cast< JointWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int dyros_jet_gui::JointWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
