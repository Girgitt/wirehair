#
# Copyleft (c) 2019 Daniel Norte de Moraes <danielcheagle@gmail.com>.
#
# * This code is hereby placed in the public domain.
# *
# * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ''AS IS'' AND ANY EXPRESS
# * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE
# * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# An working example in python using the shared lib libwirehair-shared.so .
# adjust it for your actual lib and remenber you can do much more. Enjoy!!

import os
import ctypes
import random
import string
# Success code
Wirehair_Success = 0

# More data is needed to decode.  This is normal and does not indicate a failure
Wirehair_NeedMore = 1

# Other values are failure codes:

# A function parameter was invalid
Wirehair_InvalidInput = 2

# Encoder needs a better dense seed
Wirehair_BadDenseSeed = 3

# Encoder needs a better peel seed
Wirehair_BadPeelSeed = 4

# N = ceil(messageBytes / blockBytes) is too small.
# Try reducing block_size or use a larger message
Wirehair_BadInput_SmallN = 5

# N = ceil(messageBytes / blockBytes) is too large.
# Try increasing block_size or use a smaller message
Wirehair_BadInput_LargeN = 6

# Not enough extra rows to solve it, must give up
Wirehair_ExtraInsufficient = 7

# An error occurred during the request
Wirehair_Error = 8

# Out of memory
Wirehair_OOM = 9

# Platform is not supported yet
Wirehair_UnsupportedPlatform = 10

WirehairResult_Count = 11  # /* for asserts */

WirehairResult_Padding = 0x7fffffff  # /* int32_t padding */

#wirehair = ctypes.CDLL("libwirehair-shared.so")  # MSWindows: just remove ".so" part to use DLL
wirehair = ctypes.CDLL(os.path.join(os.getcwd(), "wirehair.dll"))  # MSWindows: just remove ".so" part to use DLL

pkt_size = 26000
payload_size = 4 * 1024*1024
max_lost_pct = 0.12
max_in_row_lost = 3
redundancy_factor = 1.3

KPacketSize = ctypes.c_int(pkt_size)  # this can be extremaly large 1400 or more! :-)
'''
Message_tmp = b'A working example. this need be minimum o 2*KPacketSize; because this I in filling ' \
              b'more and more words just' \
              b'by the sake of filling... :-)  the real data can be and will be different :-) '

'''
slice_size = int(payload_size / 5)

#payload_slice = [b'%s' % random.randint(0, 255) for item in range(slice_size)]
payload_slice = [b'%s' % random.choice(string.ascii_letters) for item in range(slice_size)]

payload = []
for i in range(int(payload_size/slice_size)):
    payload.extend(payload_slice)

payload = payload[:payload_size]

#with open('./20190913.498173', 'rb') as fhi:
#with open('./.77122.1053838', 'rb') as fhi:
#with open('./third_party.zip', 'rb') as fhi:
#    payload = ''.join(fhi.readlines())

Message_tmp = ''.join(payload[:payload_size])

#Message_tmp = b'0' * payload_size


Message = (ctypes.c_uint8 * len(Message_tmp)).from_buffer_copy(Message_tmp)

if wirehair.wirehair_init_(2) != Wirehair_Success :
    # this "2" can change in future wirehair releases. :-)
    # Just updated when necessary.
    print("Wirehair_Init() failed! exiting.")
    exit()

encoder = wirehair.wirehair_encoder_create(0, ctypes.byref(Message),
                                           ctypes.c_uint64(len(Message)),
                                           ctypes.c_uint32(KPacketSize.value))

if encoder == 0:
    print("Creation of encoder failed! exiting.")
    exit()

decoder = wirehair.wirehair_decoder_create(0,
                                           ctypes.c_uint64(len(Message)),
                                           ctypes.c_uint32(KPacketSize.value))

if decoder == 0:
    print("Creation of encoder failed! exiting.")
    wirehair.wirehair_free(encoder)
    exit()

blockid = ctypes.c_uint(0)
needed = ctypes.c_uint(0)

packets_cnt_nominal = int((payload_size/pkt_size)*1.03)  # as per description 3% overhead should guarantee assembly
packets_cnt = int(packets_cnt_nominal*redundancy_factor)

max_lost_cnt = int(packets_cnt_nominal * max_lost_pct)

lost_cnt = 0
lost_in_row_cnt = 0
print("packets cnt: %s" % packets_cnt)
print("packets cnt nominal: %s" % packets_cnt_nominal)
print("packets max to loose: %s" % max_lost_cnt)
for i in range(packets_cnt):

    blockid.value += 1

    # simulate 10% packet loss
    if random.random() > 0.9:
        if lost_cnt < max_lost_cnt:
            if lost_in_row_cnt == 0:
                print("--\\")
            lost_cnt += 1
            lost_in_row_cnt += 1
            print("  |- DROPPING PACKET %s" % blockid.value)
            #pass
            continue
    if lost_in_row_cnt > 0:
        if lost_in_row_cnt < max_in_row_lost:
            if lost_cnt < max_lost_cnt:
                lost_in_row_cnt += 1
                if random.random() < 0.5:
                    lost_cnt += 1
                    print("  |- DROPPING PACKET in a row %s" % blockid.value)
                    continue
        else:
            lost_in_row_cnt = 0

    needed.value += 1
    block = (ctypes.c_uint8 * KPacketSize.value)()
    # ? They are real need to redefining it always in loop ??
    # ? Will speedup define it before the while loop, just one time?

    # Encode a packet
    writelen = ctypes.c_uint32(0)
    encodedResult = wirehair.wirehair_encode(encoder, #encoder object
                                            ctypes.c_uint(blockid.value), #ID of block to generate
                                            ctypes.byref(block), #output buffer
                                            ctypes.c_uint32(KPacketSize.value), #output buffer size
                                            ctypes.byref(writelen)) # returned block length

    if encodedResult != Wirehair_Success:
        print("Wirehair_encode failed! exiting.")
        exit()
    elif blockid.value % int(packets_cnt/20) == 0:
        print("block %s:  %s .. %s" % (blockid.value, ' '.join([str(item) for item in block[:16]]), ' '.join([str(item) for item in block[-16:]])))
        #print("block %s:  %s" % (blockid.value,  block[:64]))
    #    print(len(block))


    decodeResult = wirehair.wirehair_decode(
        decoder,  # Decoder Object
        ctypes.c_uint(blockid.value),  # ID of block that was encoded
        ctypes.byref(block),  # input buffer
        writelen  # Block length
    )

    if decodeResult == Wirehair_Success:
        # Decoder has enough data to recover now
        print("completed on packet %s/%s" % (blockid.value, packets_cnt))
        print("lost %%: %.2f/%.2f" % (float(lost_cnt)*100/packets_cnt, max_lost_pct*100))
        break

    if decodeResult != Wirehair_NeedMore:
        print("Wirehair_decode failed: ", decodeResult, " \n")



# recover original data on decoder side
for i in range(1):
    decoded = (ctypes.c_uint8 * len(Message))()
    decodeResult = wirehair.wirehair_recover(
        decoder,
        ctypes.byref(decoded),
        ctypes.c_uint64(len(Message))
    )

if decodeResult != 0:
    print("Wirehair_recover failed! exiting.")
    exit()

eita = (ctypes.c_byte * len(Message)).from_buffer_copy(bytearray(decoded[:]))

if decoded[:] == Message[:]:
    print("OK msgs are equal")
    print("in:  %s..%s" % (Message_tmp[:10], Message_tmp[-10:]))
    print("out: %s..%s" % (bytearray(eita)[:10],bytearray(eita)[-10:]))
else:
    print("MESSAGE CORRUPTED")
    # just more for fun
    #eita = (ctypes.c_byte * len(Message)).from_buffer_copy(bytearray(decoded[:]))
    #print(bytearray(eita))


# just more for fun
#eita = (ctypes.c_byte * len(Message)).from_buffer_copy(bytearray(decoded[:]))
#print(bytearray(eita))

wirehair.wirehair_free(encoder) ## ? are need to "free" from python? maybe not. :-)
wirehair.wirehair_free(decoder) ## fixme if necessary: if "wirehair.wirehair_free()" are causing trouble, remove they.

#  Obs.: tested in Linux Ubuntu Disco Jingo, gcc-8.3 and python 3.7.2+
#  in 19 march 2019

# Enjoy!! :-)