//
// Generated file, do not edit! Created by nedtool 5.7 from AppPackets.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wshadow"
#  pragma clang diagnostic ignored "-Wconversion"
#  pragma clang diagnostic ignored "-Wunused-parameter"
#  pragma clang diagnostic ignored "-Wc++98-compat"
#  pragma clang diagnostic ignored "-Wunreachable-code-break"
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wshadow"
#  pragma GCC diagnostic ignored "-Wconversion"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#  pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#  pragma GCC diagnostic ignored "-Wfloat-conversion"
#endif

#include <iostream>
#include <sstream>
#include "AppPackets_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp


// forward
template<typename T, typename A>
std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec);

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
inline std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// operator<< for std::vector<T>
template<typename T, typename A>
inline std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec)
{
    out.put('{');
    for(typename std::vector<T,A>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        if (it != vec.begin()) {
            out.put(','); out.put(' ');
        }
        out << *it;
    }
    out.put('}');
    
    char buf[32];
    sprintf(buf, " (size=%u)", (unsigned int)vec.size());
    out.write(buf, strlen(buf));
    return out;
}

Register_Class(AppPacket)

AppPacket::AppPacket(const char *name, short kind) : ::omnetpp::cPacket(name,kind)
{
    this->setName("AppPacket");

    this->genTime = 0;
    this->lastBurstPacket = false;
    this->idx = 0;
    this->idBurst = 0;
    this->video = false;
}

AppPacket::AppPacket(const AppPacket& other) : ::omnetpp::cPacket(other)
{
    copy(other);
}

AppPacket::~AppPacket()
{
}

AppPacket& AppPacket::operator=(const AppPacket& other)
{
    if (this==&other) return *this;
    ::omnetpp::cPacket::operator=(other);
    copy(other);
    return *this;
}

void AppPacket::copy(const AppPacket& other)
{
    this->genTime = other.genTime;
    this->lastBurstPacket = other.lastBurstPacket;
    this->idx = other.idx;
    this->idBurst = other.idBurst;
    this->video = other.video;
}

void AppPacket::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cPacket::parsimPack(b);
    doParsimPacking(b,this->genTime);
    doParsimPacking(b,this->lastBurstPacket);
    doParsimPacking(b,this->idx);
    doParsimPacking(b,this->idBurst);
    doParsimPacking(b,this->video);
}

void AppPacket::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cPacket::parsimUnpack(b);
    doParsimUnpacking(b,this->genTime);
    doParsimUnpacking(b,this->lastBurstPacket);
    doParsimUnpacking(b,this->idx);
    doParsimUnpacking(b,this->idBurst);
    doParsimUnpacking(b,this->video);
}

::omnetpp::simtime_t AppPacket::getGenTime() const
{
    return this->genTime;
}

void AppPacket::setGenTime(::omnetpp::simtime_t genTime)
{
    this->genTime = genTime;
}

bool AppPacket::getLastBurstPacket() const
{
    return this->lastBurstPacket;
}

void AppPacket::setLastBurstPacket(bool lastBurstPacket)
{
    this->lastBurstPacket = lastBurstPacket;
}

int AppPacket::getIdx() const
{
    return this->idx;
}

void AppPacket::setIdx(int idx)
{
    this->idx = idx;
}

int AppPacket::getIdBurst() const
{
    return this->idBurst;
}

void AppPacket::setIdBurst(int idBurst)
{
    this->idBurst = idBurst;
}

bool AppPacket::getVideo() const
{
    return this->video;
}

void AppPacket::setVideo(bool video)
{
    this->video = video;
}

class AppPacketDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    AppPacketDescriptor();
    virtual ~AppPacketDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyname) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyname) const override;
    virtual int getFieldArraySize(void *object, int field) const override;

    virtual const char *getFieldDynamicTypeString(void *object, int field, int i) const override;
    virtual std::string getFieldValueAsString(void *object, int field, int i) const override;
    virtual bool setFieldValueAsString(void *object, int field, int i, const char *value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual void *getFieldStructValuePointer(void *object, int field, int i) const override;
};

Register_ClassDescriptor(AppPacketDescriptor)

AppPacketDescriptor::AppPacketDescriptor() : omnetpp::cClassDescriptor("AppPacket", "omnetpp::cPacket")
{
    propertynames = nullptr;
}

AppPacketDescriptor::~AppPacketDescriptor()
{
    delete[] propertynames;
}

bool AppPacketDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<AppPacket *>(obj)!=nullptr;
}

const char **AppPacketDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *AppPacketDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int AppPacketDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 5+basedesc->getFieldCount() : 5;
}

unsigned int AppPacketDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeFlags(field);
        field -= basedesc->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<5) ? fieldTypeFlags[field] : 0;
}

const char *AppPacketDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "genTime",
        "lastBurstPacket",
        "idx",
        "idBurst",
        "video",
    };
    return (field>=0 && field<5) ? fieldNames[field] : nullptr;
}

int AppPacketDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='g' && strcmp(fieldName, "genTime")==0) return base+0;
    if (fieldName[0]=='l' && strcmp(fieldName, "lastBurstPacket")==0) return base+1;
    if (fieldName[0]=='i' && strcmp(fieldName, "idx")==0) return base+2;
    if (fieldName[0]=='i' && strcmp(fieldName, "idBurst")==0) return base+3;
    if (fieldName[0]=='v' && strcmp(fieldName, "video")==0) return base+4;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *AppPacketDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeString(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "simtime_t",
        "bool",
        "int",
        "int",
        "bool",
    };
    return (field>=0 && field<5) ? fieldTypeStrings[field] : nullptr;
}

const char **AppPacketDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldPropertyNames(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *AppPacketDescriptor::getFieldProperty(int field, const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldProperty(field, propertyname);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int AppPacketDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    AppPacket *pp = (AppPacket *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

const char *AppPacketDescriptor::getFieldDynamicTypeString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldDynamicTypeString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    AppPacket *pp = (AppPacket *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string AppPacketDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    AppPacket *pp = (AppPacket *)object; (void)pp;
    switch (field) {
        case 0: return simtime2string(pp->getGenTime());
        case 1: return bool2string(pp->getLastBurstPacket());
        case 2: return long2string(pp->getIdx());
        case 3: return long2string(pp->getIdBurst());
        case 4: return bool2string(pp->getVideo());
        default: return "";
    }
}

bool AppPacketDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    AppPacket *pp = (AppPacket *)object; (void)pp;
    switch (field) {
        case 0: pp->setGenTime(string2simtime(value)); return true;
        case 1: pp->setLastBurstPacket(string2bool(value)); return true;
        case 2: pp->setIdx(string2long(value)); return true;
        case 3: pp->setIdBurst(string2long(value)); return true;
        case 4: pp->setVideo(string2bool(value)); return true;
        default: return false;
    }
}

const char *AppPacketDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructName(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

void *AppPacketDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    AppPacket *pp = (AppPacket *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

Register_Class(AppControlInfo)

AppControlInfo::AppControlInfo(const char *name, short kind) : ::omnetpp::cMessage(name,kind)
{
    this->priority = 0;
}

AppControlInfo::AppControlInfo(const AppControlInfo& other) : ::omnetpp::cMessage(other)
{
    copy(other);
}

AppControlInfo::~AppControlInfo()
{
}

AppControlInfo& AppControlInfo::operator=(const AppControlInfo& other)
{
    if (this==&other) return *this;
    ::omnetpp::cMessage::operator=(other);
    copy(other);
    return *this;
}

void AppControlInfo::copy(const AppControlInfo& other)
{
    this->dst = other.dst;
    this->src = other.src;
    this->priority = other.priority;
}

void AppControlInfo::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cMessage::parsimPack(b);
    doParsimPacking(b,this->dst);
    doParsimPacking(b,this->src);
    doParsimPacking(b,this->priority);
}

void AppControlInfo::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cMessage::parsimUnpack(b);
    doParsimUnpacking(b,this->dst);
    doParsimUnpacking(b,this->src);
    doParsimUnpacking(b,this->priority);
}

const char * AppControlInfo::getDst() const
{
    return this->dst.c_str();
}

void AppControlInfo::setDst(const char * dst)
{
    this->dst = dst;
}

const char * AppControlInfo::getSrc() const
{
    return this->src.c_str();
}

void AppControlInfo::setSrc(const char * src)
{
    this->src = src;
}

::omnetpp::simtime_t AppControlInfo::getPriority() const
{
    return this->priority;
}

void AppControlInfo::setPriority(::omnetpp::simtime_t priority)
{
    this->priority = priority;
}

class AppControlInfoDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    AppControlInfoDescriptor();
    virtual ~AppControlInfoDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyname) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyname) const override;
    virtual int getFieldArraySize(void *object, int field) const override;

    virtual const char *getFieldDynamicTypeString(void *object, int field, int i) const override;
    virtual std::string getFieldValueAsString(void *object, int field, int i) const override;
    virtual bool setFieldValueAsString(void *object, int field, int i, const char *value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual void *getFieldStructValuePointer(void *object, int field, int i) const override;
};

Register_ClassDescriptor(AppControlInfoDescriptor)

AppControlInfoDescriptor::AppControlInfoDescriptor() : omnetpp::cClassDescriptor("AppControlInfo", "omnetpp::cMessage")
{
    propertynames = nullptr;
}

AppControlInfoDescriptor::~AppControlInfoDescriptor()
{
    delete[] propertynames;
}

bool AppControlInfoDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<AppControlInfo *>(obj)!=nullptr;
}

const char **AppControlInfoDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *AppControlInfoDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int AppControlInfoDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 3+basedesc->getFieldCount() : 3;
}

unsigned int AppControlInfoDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeFlags(field);
        field -= basedesc->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<3) ? fieldTypeFlags[field] : 0;
}

const char *AppControlInfoDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "dst",
        "src",
        "priority",
    };
    return (field>=0 && field<3) ? fieldNames[field] : nullptr;
}

int AppControlInfoDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='d' && strcmp(fieldName, "dst")==0) return base+0;
    if (fieldName[0]=='s' && strcmp(fieldName, "src")==0) return base+1;
    if (fieldName[0]=='p' && strcmp(fieldName, "priority")==0) return base+2;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *AppControlInfoDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeString(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "string",
        "string",
        "simtime_t",
    };
    return (field>=0 && field<3) ? fieldTypeStrings[field] : nullptr;
}

const char **AppControlInfoDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldPropertyNames(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *AppControlInfoDescriptor::getFieldProperty(int field, const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldProperty(field, propertyname);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int AppControlInfoDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    AppControlInfo *pp = (AppControlInfo *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

const char *AppControlInfoDescriptor::getFieldDynamicTypeString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldDynamicTypeString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    AppControlInfo *pp = (AppControlInfo *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string AppControlInfoDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    AppControlInfo *pp = (AppControlInfo *)object; (void)pp;
    switch (field) {
        case 0: return oppstring2string(pp->getDst());
        case 1: return oppstring2string(pp->getSrc());
        case 2: return simtime2string(pp->getPriority());
        default: return "";
    }
}

bool AppControlInfoDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    AppControlInfo *pp = (AppControlInfo *)object; (void)pp;
    switch (field) {
        case 0: pp->setDst((value)); return true;
        case 1: pp->setSrc((value)); return true;
        case 2: pp->setPriority(string2simtime(value)); return true;
        default: return false;
    }
}

const char *AppControlInfoDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructName(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

void *AppControlInfoDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    AppControlInfo *pp = (AppControlInfo *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}


