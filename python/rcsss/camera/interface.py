from abc import ABC, abstractmethod
from dataclasses import dataclass
from datetime import datetime
from typing import Generic, Literal, TypeVar

import numpy as np
from pydantic import BaseModel, ConfigDict


class BaseDataClass(BaseModel):
    model_config = ConfigDict(
        arbitrary_types_allowed=True,
    )


class BaseCameraConfig(BaseDataClass): ...

# DType = TypeVar('DType') 
# Generic[DType], 
@dataclass
class DataFrame():
    data: np.ndarray
    timestamp: float | None = None

class CameraFrame(BaseDataClass):
    color: DataFrame | None = None
    ir: DataFrame | None = None
    depth: DataFrame | None = None
    temperature: float | None = None


class IMUFrame(BaseDataClass):
    accel: DataFrame | None = None
    gyro: DataFrame | None = None
    temperature: float | None = None


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

