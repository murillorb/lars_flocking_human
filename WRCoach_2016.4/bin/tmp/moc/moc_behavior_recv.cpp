/****************************************************************************
** Meta object code from reading C++ file 'behavior_recv.hh'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/WRCoach/entity/player/behavior/ssl/behavior_recv.hh"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'behavior_recv.hh' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Behavior_Recv_t {
    QByteArrayData data[7];
    char stringdata[86];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Behavior_Recv_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Behavior_Recv_t qt_meta_stringdata_Behavior_Recv = {
    {
QT_MOC_LITERAL(0, 0, 13), // "Behavior_Recv"
QT_MOC_LITERAL(1, 14, 12), // "ballReceived"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 6), // "recvID"
QT_MOC_LITERAL(4, 35, 15), // "ballNotReceived"
QT_MOC_LITERAL(5, 51, 19), // "attackerAboutToKick"
QT_MOC_LITERAL(6, 71, 14) // "attackerKicked"

    },
    "Behavior_Recv\0ballReceived\0\0recvID\0"
    "ballNotReceived\0attackerAboutToKick\0"
    "attackerKicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Behavior_Recv[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       4,    1,   37,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   40,    2, 0x0a /* Public */,
       6,    1,   43,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::UChar,    3,
    QMetaType::Void, QMetaType::UChar,    3,

 // slots: parameters
    QMetaType::Void, QMetaType::UChar,    3,
    QMetaType::Void, QMetaType::UChar,    3,

       0        // eod
};

void Behavior_Recv::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Behavior_Recv *_t = static_cast<Behavior_Recv *>(_o);
        switch (_id) {
        case 0: _t->ballReceived((*reinterpret_cast< quint8(*)>(_a[1]))); break;
        case 1: _t->ballNotReceived((*reinterpret_cast< quint8(*)>(_a[1]))); break;
        case 2: _t->attackerAboutToKick((*reinterpret_cast< quint8(*)>(_a[1]))); break;
        case 3: _t->attackerKicked((*reinterpret_cast< quint8(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (Behavior_Recv::*_t)(quint8 );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Behavior_Recv::ballReceived)) {
                *result = 0;
            }
        }
        {
            typedef void (Behavior_Recv::*_t)(quint8 );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Behavior_Recv::ballNotReceived)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject Behavior_Recv::staticMetaObject = {
    { &Behavior::staticMetaObject, qt_meta_stringdata_Behavior_Recv.data,
      qt_meta_data_Behavior_Recv,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Behavior_Recv::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Behavior_Recv::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Behavior_Recv.stringdata))
        return static_cast<void*>(const_cast< Behavior_Recv*>(this));
    return Behavior::qt_metacast(_clname);
}

int Behavior_Recv::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Behavior::qt_metacall(_c, _id, _a);
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

// SIGNAL 0
void Behavior_Recv::ballReceived(quint8 _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Behavior_Recv::ballNotReceived(quint8 _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
