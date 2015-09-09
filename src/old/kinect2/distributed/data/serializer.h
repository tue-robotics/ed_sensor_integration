#ifndef ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_SERIALIZER_H_
#define ED_SENSOR_INTEGRATION_KINECT2_DISTRIBUTED_DATA_SERIALIZER_H_

#include <blackboard/serializer.h>

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include <sstream>

class ArchiveSerializer : public bb::Serializer
{

    virtual void serialize(const bb::Variant& v, tue::serialization::OutputArchive& a) = 0;

    virtual void deserialize(tue::serialization::InputArchive& a, bb::Variant& v) = 0;

    void serialize(const bb::Variant& v, bb::WBytes& bytes)
    {
        std::stringstream stream;
        tue::serialization::OutputArchive a(stream);

        serialize(v, a);

        // get its size:
        stream.seekg(0, std::ios::end);
        int size = stream.tellg();
        stream.seekg(0, std::ios::beg);

        bytes.resize(size);
        stream.read((char*)bytes.ptr(), size);
    }

    void deserialize(const bb::RBytes& bytes, bb::Variant& v)
    {
        std::stringstream s;
        const char* data = reinterpret_cast<const char*>(bytes.ptr());
        s.write(data, bytes.size());

        tue::serialization::InputArchive a(s);

        // -----------------

        deserialize(a, v);
    }
};

#endif
