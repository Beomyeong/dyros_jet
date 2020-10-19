/****************************************************************************
** Meta object code from reading C++ file 'task_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/dyros_jet/dyros_jet_gui/include/dyros_jet_gui/task_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'task_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_dyros_jet_gui__TaskWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      22,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      27,   26,   26,   26, 0x0a,
      54,   26,   26,   26, 0x0a,
      85,   26,   26,   26, 0x0a,
     125,   26,   26,   26, 0x0a,
     165,   26,   26,   26, 0x0a,
     195,   26,   26,   26, 0x0a,
     224,   26,   26,   26, 0x0a,
     253,   26,   26,   26, 0x0a,
     286,   26,   26,   26, 0x0a,
     323,   26,   26,   26, 0x0a,
     367,   26,   26,   26, 0x0a,
     409,   26,   26,   26, 0x0a,
     442,   26,   26,   26, 0x0a,
     474,   26,   26,   26, 0x0a,
     507,   26,   26,   26, 0x0a,
     527,  521,   26,   26, 0x0a,
     557,  521,   26,   26, 0x0a,
     592,  587,   26,   26, 0x0a,
     636,  587,   26,   26, 0x0a,
     680,   26,   26,   26, 0x0a,
     703,   26,   26,   26, 0x0a,
     725,   26,   26,   26, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_dyros_jet_gui__TaskWindow[] = {
    "dyros_jet_gui::TaskWindow\0\0"
    "on_actionAbout_triggered()\0"
    "on_button_walk_start_clicked()\0"
    "on_button_walk_init_wholebody_clicked()\0"
    "on_button_walk_init_lowerbody_clicked()\0"
    "on_button_walk_stop_clicked()\0"
    "on_button_joy_walk_clicked()\0"
    "on_button_ft_calib_clicked()\0"
    "on_button_hand_command_clicked()\0"
    "on_button_hand_preset_open_clicked()\0"
    "on_button_hand_preset_drill_power_clicked()\0"
    "on_button_hand_preset_box_pinch_clicked()\0"
    "on_pushButton_ft_start_clicked()\0"
    "on_pushButton_ft_stop_clicked()\0"
    "on_pushButton_ft_reset_clicked()\0"
    "UpdateGraph()\0event\0ClickedGraph_LF(QMouseEvent*)\0"
    "ClickedGraph_RF(QMouseEvent*)\0arg1\0"
    "on_comboBox_LF_currentIndexChanged(QString)\0"
    "on_comboBox_RF_currentIndexChanged(QString)\0"
    "taskCtrlMinusClicked()\0taskCtrlPlusClicked()\0"
    "on_pushButton_task_option_clicked()\0"
};

void dyros_jet_gui::TaskWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        TaskWindow *_t = static_cast<TaskWindow *>(_o);
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_button_walk_start_clicked(); break;
        case 2: _t->on_button_walk_init_wholebody_clicked(); break;
        case 3: _t->on_button_walk_init_lowerbody_clicked(); break;
        case 4: _t->on_button_walk_stop_clicked(); break;
        case 5: _t->on_button_joy_walk_clicked(); break;
        case 6: _t->on_button_ft_calib_clicked(); break;
        case 7: _t->on_button_hand_command_clicked(); break;
        case 8: _t->on_button_hand_preset_open_clicked(); break;
        case 9: _t->on_button_hand_preset_drill_power_clicked(); break;
        case 10: _t->on_button_hand_preset_box_pinch_clicked(); break;
        case 11: _t->on_pushButton_ft_start_clicked(); break;
        case 12: _t->on_pushButton_ft_stop_clicked(); break;
        case 13: _t->on_pushButton_ft_reset_clicked(); break;
        case 14: _t->UpdateGraph(); break;
        case 15: _t->ClickedGraph_LF((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 16: _t->ClickedGraph_RF((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 17: _t->on_comboBox_LF_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 18: _t->on_comboBox_RF_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 19: _t->taskCtrlMinusClicked(); break;
        case 20: _t->taskCtrlPlusClicked(); break;
        case 21: _t->on_pushButton_task_option_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData dyros_jet_gui::TaskWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject dyros_jet_gui::TaskWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_dyros_jet_gui__TaskWindow,
      qt_meta_data_dyros_jet_gui__TaskWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &dyros_jet_gui::TaskWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *dyros_jet_gui::TaskWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *dyros_jet_gui::TaskWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_dyros_jet_gui__TaskWindow))
        return static_cast<void*>(const_cast< TaskWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int dyros_jet_gui::TaskWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 22)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 22;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
