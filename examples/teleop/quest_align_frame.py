from franka import MQ3_ADDR
from rcs.operator.quest import FakeSimPublisher, FakeSimScene
from simpub.xr_device.meta_quest3 import MetaQuest3

publisher = FakeSimPublisher(FakeSimScene(), MQ3_ADDR)
reader = MetaQuest3("RCSNode")
while True:
    data = reader.get_controller_data()
    # print(data)
