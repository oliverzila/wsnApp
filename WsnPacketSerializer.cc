//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "inet/applications/wsnApp/WsnPacket_m.h"
#include "inet/applications/wsnApp/WsnPacketSerializer.h"
#include "inet/common/packet/serializer/ChunkSerializerRegistry.h"

namespace inet {

Register_Serializer(WsnPacket, WsnPacketSerializer);

void WsnPacketSerializer::serialize(MemoryOutputStream& stream, const Ptr<const Chunk>& chunk) const
{
    auto startPosition = stream.getLength();
    const auto& wsnPacket = staticPtrCast<const WsnPacket>(chunk);
    stream.writeUint32Be(B(wsnPacket->getChunkLength()).get());
    stream.writeUint32Be(wsnPacket->getSequenceNumber());
    stream.writeUint64Be(wsnPacket->getXPosition());
    stream.writeUint64Be(wsnPacket->getYPosition());
    int64_t remainders = B(wsnPacket->getChunkLength() - (stream.getLength() - startPosition)).get();
    if (remainders < 0)
        throw cRuntimeError("WsnPacket length = %d smaller than required %d bytes", (int)B(wsnPacket->getChunkLength()).get(), (int)B(stream.getLength() - startPosition).get());
    stream.writeByteRepeatedly('?', remainders);
}

const Ptr<Chunk> WsnPacketSerializer::deserialize(MemoryInputStream& stream) const
{
    auto startPosition = stream.getPosition();
    auto wsnPacket = makeShared<WsnPacket>();
    B dataLength = B(stream.readUint32Be());
    wsnPacket->setSequenceNumber(stream.readUint32Be());
    wsnPacket->setXPosition(stream.readUint64Be());
    wsnPacket->setYPosition(stream.readUint64Be());
    B remainders = dataLength - (stream.getPosition() - startPosition);
    ASSERT(remainders >= B(0));
    stream.readByteRepeatedly('?', B(remainders).get());
    return wsnPacket;
}

} // namespace inet

