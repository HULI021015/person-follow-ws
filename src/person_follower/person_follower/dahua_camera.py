"""Low-latency Dahua NetSDK camera capture using the bundled PlaySDK decoder."""

from __future__ import annotations

import threading
import time
from ctypes import POINTER, c_int, c_ubyte, c_uint, cast, sizeof
from dataclasses import dataclass
from typing import Callable, Optional

import cv2
import numpy as np

from NetSDK.NetSDK import NetClient
from NetSDK.SDK_Callback import (
    fDecCBFun,
    fDisConnect,
    fHaveReConnect,
    fRealDataCallBackEx2,
)
from NetSDK.SDK_Enum import EM_LOGIN_SPAC_CAP_TYPE, EM_REALDATA_FLAG, SDK_RealPlayType
from NetSDK.SDK_Struct import (
    NET_IN_LOGIN_WITH_HIGHLEVEL_SECURITY,
    NET_OUT_LOGIN_WITH_HIGHLEVEL_SECURITY,
)


_STREAM_TYPES = {
    "main": SDK_RealPlayType.Realplay,
    "extra": SDK_RealPlayType.Realplay_1,
}


def _integer_value(value) -> int:
    """Accept either a ctypes scalar or the Python int produced by CFUNCTYPE."""
    return int(value.value if hasattr(value, "value") else value)


@dataclass(frozen=True)
class CameraFrame:
    image: np.ndarray
    version: int
    received_monotonic: float
    sdk_timestamp_ms: int
    source_fps: int


class DahuaCamera:
    """Decode Dahua real-time video and retain only the most recent frame."""

    def __init__(
        self,
        ip: str,
        port: int,
        username: str,
        password: str,
        channel: int = 0,
        stream: str = "extra",
        event_callback: Optional[Callable[[str, bool], None]] = None,
    ) -> None:
        if stream not in _STREAM_TYPES:
            raise ValueError("stream must be 'main' or 'extra'")
        if not password:
            raise ValueError("camera password is empty; set DAHUA_PASSWORD or camera_password")

        self.ip = ip
        self.port = int(port)
        self.username = username
        self.password = password
        self.channel = int(channel)
        self.stream_type = _STREAM_TYPES[stream]
        self.event_callback = event_callback

        self.login_id = 0
        self.play_id = 0
        self.free_port = c_int(-1)
        self.connected = False
        self.started = False
        self._closed = False
        self._latest_yuv = None
        self._version = 0
        self._input_drops = 0
        self._lock = threading.Lock()

        # ctypes callbacks must remain strongly referenced for the whole stream lifetime.
        self._disconnect_cb = fDisConnect(self._on_disconnect)
        self._reconnect_cb = fHaveReConnect(self._on_reconnect)
        self._real_data_cb = fRealDataCallBackEx2(self._on_real_data)
        self._decode_cb = fDecCBFun(self._on_decode)

        self.sdk = NetClient()
        if not hasattr(self.sdk, "sdk") or not hasattr(self.sdk, "play_sdk"):
            raise RuntimeError(
                "NetSDK dynamic libraries did not load. Source scripts/netsdk_env.sh "
                "before starting ROS 2."
            )
        if self.sdk.InitEx(self._disconnect_cb) != 1:
            raise RuntimeError("NetSDK initialization failed")
        self.sdk.SetAutoReconnect(self._reconnect_cb)

    def _emit(self, message: str, is_error: bool = False) -> None:
        if self.event_callback is not None:
            self.event_callback(message, is_error)

    def _on_disconnect(self, _login_id, _ip, _port, _user) -> None:
        self.connected = False
        self._emit("大华相机连接断开，控制将停车", True)

    def _on_reconnect(self, _login_id, _ip, _port, _user) -> None:
        self.connected = True
        self._emit("大华相机已由 NetSDK 自动重连", False)

    def _on_real_data(self, real_handle, _data_type, buffer, size, _param, _user) -> None:
        if not self.started or _integer_value(real_handle) != _integer_value(self.play_id):
            return
        if not self.sdk.InputData(self.free_port, buffer, size):
            with self._lock:
                self._input_drops += 1

    def _on_decode(self, _port, buffer, size, frame_info, _user, _reserved) -> None:
        if not self.started or not frame_info:
            return
        info = frame_info.contents
        if info.nType != 3 or info.nWidth <= 0 or info.nHeight <= 0:
            return
        expected = info.nWidth * info.nHeight * 3 // 2
        if size < expected:
            return
        raw = cast(buffer, POINTER(c_ubyte * expected)).contents
        item = (
            bytes(raw),
            int(info.nWidth),
            int(info.nHeight),
            time.monotonic(),
            int(info.nStamp),
            int(info.nFrameRate),
        )
        with self._lock:
            self._latest_yuv = item
            self._version += 1

    def login(self) -> None:
        login_input = NET_IN_LOGIN_WITH_HIGHLEVEL_SECURITY()
        login_input.dwSize = sizeof(NET_IN_LOGIN_WITH_HIGHLEVEL_SECURITY)
        login_input.szIP = self.ip.encode()
        login_input.nPort = self.port
        login_input.szUserName = self.username.encode()
        login_input.szPassword = self.password.encode()
        login_input.emSpecCap = EM_LOGIN_SPAC_CAP_TYPE.TCP
        login_input.pCapParam = None

        login_output = NET_OUT_LOGIN_WITH_HIGHLEVEL_SECURITY()
        login_output.dwSize = sizeof(NET_OUT_LOGIN_WITH_HIGHLEVEL_SECURITY)
        self.login_id, device_info, error = self.sdk.LoginWithHighLevelSecurity(
            login_input, login_output
        )
        if not self.login_id:
            raise RuntimeError(f"大华相机登录失败：{error or self.sdk.GetLastErrorMessage()}")
        self.connected = True
        self._emit(f"大华相机登录成功，设备通道数 {int(device_info.nChanNum)}", False)

    def start(self) -> None:
        if not self.login_id:
            raise RuntimeError("camera must be logged in before start")

        ok, self.free_port = self.sdk.GetFreePort()
        if not ok:
            raise RuntimeError("PlaySDK could not allocate a decoder port")

        play_sdk = self.sdk.play_sdk
        play_sdk.PLAY_SetStreamOpenMode.argtypes = [c_int, c_uint]
        play_sdk.PLAY_SetStreamOpenMode.restype = c_int
        play_sdk.PLAY_SetDisplayBuf.argtypes = [c_int, c_uint]
        play_sdk.PLAY_SetDisplayBuf.restype = c_int
        if not play_sdk.PLAY_SetStreamOpenMode(self.free_port, 0):
            raise RuntimeError("PlaySDK could not enable real-time stream mode")
        if not self.sdk.OpenStream(self.free_port):
            raise RuntimeError("PlaySDK could not open the input stream")
        if not self.sdk.SetDecCallBack(self.free_port, self._decode_cb):
            raise RuntimeError("PlaySDK could not register the decode callback")
        if not self.sdk.Play(self.free_port, 0):
            raise RuntimeError("PlaySDK could not start headless decoding")
        # Keep PlaySDK's display buffer to one frame. The ROS side also stores
        # only one latest frame, so slow inference cannot create a historical queue.
        play_sdk.PLAY_SetDisplayBuf(self.free_port, 1)

        self.play_id = self.sdk.RealPlayEx(
            self.login_id, self.channel, 0, self.stream_type
        )
        if not self.play_id:
            raise RuntimeError(f"大华实时预览失败：{self.sdk.GetLastErrorMessage()}")
        self.started = True
        if not self.sdk.SetRealDataCallBackEx2(
            self.play_id,
            self._real_data_cb,
            None,
            EM_REALDATA_FLAG.RAW_DATA,
        ):
            raise RuntimeError("NetSDK could not register the raw-stream callback")

    def read_latest(self, after_version: int = -1) -> Optional[CameraFrame]:
        """Return the newest decoded frame, or None if no newer frame exists."""
        with self._lock:
            version = self._version
            item = self._latest_yuv
        if item is None or version == after_version:
            return None
        raw, width, height, received, sdk_stamp, source_fps = item
        yuv = np.frombuffer(raw, dtype=np.uint8).reshape((height * 3 // 2, width))
        image = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        return CameraFrame(image, version, received, sdk_stamp, source_fps)

    def status(self) -> tuple[int, Optional[float], int]:
        with self._lock:
            received = None if self._latest_yuv is None else self._latest_yuv[3]
            return self._version, received, self._input_drops

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.started = False
        if self.play_id:
            try:
                self.sdk.StopRealPlayEx(self.play_id)
            finally:
                self.play_id = 0
        if self.free_port.value >= 0:
            try:
                self.sdk.Stop(self.free_port)
                self.sdk.CloseStream(self.free_port)
                self.sdk.ReleasePort(self.free_port)
            finally:
                self.free_port = c_int(-1)
        if self.login_id:
            try:
                self.sdk.Logout(self.login_id)
            finally:
                self.login_id = 0
        self.connected = False
        self.sdk.Cleanup()
