/****************************************************************************
** Meta object code from reading C++ file 'status_qnode.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/dyros_jet/dyros_jet_gui/include/dyros_jet_gui/status_qnode.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'status_qnode.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_dyros_jet_gui__StatusQNode[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      28,   27,   27,   27, 0x05,
      43,   27,   27,   27, 0x05,
      62,   27,   27,   27, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_dyros_jet_gui__StatusQNode[] = {
    "dyros_jet_gui::StatusQNode\0\0stateUpdated()\0"
    "recogInfoUpdated()\0rosShutdown()\0"
};

void dyros_jet_gui::StatusQNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        StatusQNode *_t = static_cast<StatusQNode *>(_o);
        switch (_id) {
        case 0: _t->stateUpdated(); break;
        case 1: _t->recogInfoUpdated(); break;
        case 2: _t->rosShutdown(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData dyros_jet_gui::StatusQNode::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject dyros_jet_gui::StatusQNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_dyros_jet_gui__StatusQNode,
      qt_meta_data_dyros_jet_gui__StatusQNode, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &dyros_jet_gui::StatusQNode::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *dyros_jet_gui::StatusQNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *dyros_jet_gui::StatusQNode::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_dyros_jet_gui__StatusQNode))
        return static_cast<void*>(const_cast< StatusQNode*>(this));
    return QThread::qt_metacast(_clname);
}

int dyros_jet_gui::StatusQNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void dyros_jet_gui::StatusQNode::stateUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void dyros_jet_gui::StatusQNode::recogInfoUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void dyros_jet_gui::StatusQNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
