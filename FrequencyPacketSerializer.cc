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

#include "inet/applications/wsnApp/FrequencyPacket_m.h"
#include "inet/applications/wsnApp/FrequencyPacketSerializer.h"
#include "inet/common/packet/serializer/ChunkSerializerRegistry.h"

namespace inet {

Register_Serializer(FrequencyPacket, FrequencyPacketSerializer);

void FrequencyPacketSerializer::serialize(MemoryOutputStream& stream, const Ptr<const Chunk>& chunk) const
{
    auto startPosition = stream.getLength();
    const auto& frequencyPacket = staticPtrCast<const FrequencyPacket>(chunk);
    stream.writeUint32Be(B(frequencyPacket->getChunkLength()).get());
    stream.writeUint8(frequencyPacket->getFreqChannel());
    int64_t remainders = B(frequencyPacket->getChunkLength() - (stream.getLength() - startPosition)).get();
    if (remainders < 0)
        throw cRuntimeError("FrequencyPacket length = %d smaller than required %d bytes", (int)B(frequencyPacket->getChunkLength()).get(), (int)B(stream.getLength() - startPosition).get());
    stream.writeByteRepeatedly('?', remainders);
}

const Ptr<Chunk> FrequencyPacketSerializer::deserialize(MemoryInputStream& stream) const
{
    auto startPosition = stream.getPosition();
    auto frequencyPacket = makeShared<FrequencyPacket>();
    B dataLength = B(stream.readUint32Be());
    frequencyPacket->setFreqChannel(stream.readUint8());
    B remainders = dataLength - (stream.getPosition() - startPosition);
    ASSERT(remainders >= B(0));
    stream.readByteRepeatedly('?', B(remainders).get());
    return frequencyPacket;
}

} // namespace inet

