import struct,cv2,numpy as np
def decode_depth(message):
    """ROS compressedDepth: 12-byte config then lossless PNG.

    32FC1 uses inverse-depth quantization; 16UC1 stores millimetres directly.
    """
    data = bytes(message.data)
    if data[12:20] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('Expected compressedDepth PNG with 12-byte header')
    raw = cv2.imdecode(np.frombuffer(data[12:], np.uint8), cv2.IMREAD_UNCHANGED)
    if raw is None or raw.dtype != np.uint16:
        raise ValueError('Invalid compressedDepth PNG')
    if message.format.startswith('32FC1'):
        (_, a, b) = struct.unpack('<iff', data[:12])
        valid = (raw > 0) & (raw.astype(np.float32) > b)
        depth = np.full(raw.shape, np.nan, np.float32)
        depth[valid] = a / (raw[valid].astype(np.float32) - b)
        return depth
    if message.format.startswith('16UC1'):
        depth = raw.astype(np.float32) * 0.001
        depth[raw == 0] = np.nan
        return depth
    raise ValueError('Unsupported compressedDepth format: ' + message.format)
