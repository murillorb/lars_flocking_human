/****************************************************************************
** Meta object code from reading C++ file 'playbook_attack.hh'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/WRCoach/entity/controlmodule/coach/playbook/ssl/playbook_attack.hh"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'playbook_attack.hh' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Playbook_Attack_t {
    QByteArrayData data[7];
    char stringdata[84];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Playbook_Attack_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Playbook_Attack_t qt_meta_stringdata_Playbook_Attack = {
    {
QT_MOC_LITERAL(0, 0, 15), // "Playbook_Attack"
QT_MOC_LITERAL(1, 16, 10), // "ballKicked"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 6), // "recvID"
QT_MOC_LITERAL(4, 35, 12), // "ballReceived"
QT_MOC_LITERAL(5, 48, 15), // "ballNotReceived"
QT_MOC_LITERAL(6, 64, 19) // "attackerAboutToKick"

    },
    "Playbook_Attack\0ballKicked\0\0recvID\0"
    "ballReceived\0ballNotReceived\0"
    "attackerAboutToKick"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Playbook_Attack[] = {

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
       1,    1,   34,    2, 0x0a /* Public */,
       4,    0,   37,    2, 0x0a /* Public */,
       5,    0,   38,    2, 0x0a /* Public */,
       6,    1,   39,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::UChar,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::UChar,    3,

       0        // eod
};

void Playbook_Attack::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Playbook_Attack *_t = static_cast<Playbook_Attack *>(_o);
        switch (_id) {
        case 0: _t->ballKicked((*reinterpret_cast< quint8(*)>(_a[1]))); break;
        case 1: _t->ballReceived(); break;
        case 2: _t->ballNotReceived(); break;
        case 3: _t->attackerAboutToKick((*reinterpret_cast< quint8(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject Playbook_Attack::staticMetaObject = {
    { &Playbook::staticMetaObject, qt_meta_stringdata_Playbook_Attack.data,
      qt_meta_data_Playbook_Attack,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Playbook_Attack::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Playbook_Attack::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Playbook_Attack.stringdata))
        return static_cast<void*>(const_cast< Playbook_Attack*>(this));
    return Playbook::qt_metacast(_clname);
}

int Playbook_Attack::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Playbook::qt_metacall(_c, _id, _a);
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
