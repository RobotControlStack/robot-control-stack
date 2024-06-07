from abc import ABC, abstractmethod
from datetime import datetime
from typing import Literal, TypeVar

import numpy as np
from pydantic import BaseModel, ConfigDict


class BaseDataClass(BaseModel):
    model_config = ConfigDict(
        arbitrary_types_allowed=True,
    )


class BaseCameraConfig(BaseDataClass): ...

DType = TypeVar('DType') 
class DataFrame(BaseDataClass):
    data: DType
    timestamp: float | None

class CameraFrame(BaseDataClass):
    color: DataFrame[np.ndarray]
    ir: DataFrame[np.ndarray] | None
    depth: DataFrame[np.ndarray] | None
    temperature: float | None


class IMUFrame(BaseDataClass):
    accel: DataFrame[np.ndarray[Literal[3], np.dtype[np.float32]]]
    gyro: DataFrame[np.ndarray[Literal[3], np.dtype[np.float32]]]
    temperature: float | None


class Frame(BaseDataClass):
    camera: CameraFrame
    imu: IMUFrame | None
    avg_timestamp: float | None


class BaseCameraSet(ABC):

    @property
    @abstractmethod
    def config(self) -> BaseCameraConfig:
        """Should return the configuration object of the cameras.
        """
        

    @abstractmethod
    def get_frame_latest(self, camera_name: str) -> Frame:
        """Should return the latest frame from the camera with the given name.
        """

    @abstractmethod
    def get_frame_timestamp(self, camera_name: str, ts: datetime) -> Frame:
        """Should return the frame from the camera with the given name and closest to the given timestamp.
        """

