from abc import ABC, abstractmethod
from typing import Literal, Optional

import numpy as np
from pydantic import BaseModel


class GenericCameraConfig(BaseModel): ...


class GenericIMUConfig(BaseModel): ...


class CameraFrame(BaseModel):
    color: np.ndarray
    ir: Optional[np.ndarray]
    depth: Optional[np.ndarray]
    temperature: Optional[float]


class IMUFrame(BaseModel):
    acc_sample: np.ndarray[Literal[3], np.dtype[np.float32]]
    acc_sample_usec: Optional[float]
    gyro_sample: np.ndarray[Literal[3], np.dtype[np.float32]]
    gyro_sample_usec: Optional[float]
    temperature: Optional[float]


class Frame(BaseModel):
    camera: CameraFrame
    imu: Optional[IMUFrame]


class Camera(ABC):

    @property
    @abstractmethod
    def config(self) -> GenericCameraConfig: ...

    @config.setter
    @abstractmethod
    def config(self, cfg: GenericCameraConfig) -> None: ...

    @abstractmethod
    def get_current_frame(self) -> Frame: ...
