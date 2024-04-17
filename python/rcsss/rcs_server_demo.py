import rcsss
import rpyc
from rcsss.rpc.rcs_service import RCSService

MODEL_DIR = "../../models"

if __name__ == "__main__":
    robot = rcsss.sim.FR3(MODEL_DIR + "/mjcf/scene.xml", MODEL_DIR + "/urdf/fr3_from_panda.urdf", True)
    service = RCSService([robot])
    t = rpyc.ThreadedServer(service, port=18861, protocol_config= {"allow_pickle": True})
    t.start()
