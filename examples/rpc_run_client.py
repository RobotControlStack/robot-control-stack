import time

from python.rcs.rpc.client import RcsClient

if __name__ == "__main__":
    # Create the client (adjust host/port if needed)
    client = RcsClient(host="localhost", port=50051)

    try:
        print("Resetting environment...")
        obs = client.reset()
        print(f"Initial observation: {obs}")

        for i in range(5):
            print(f"\nStep {i+1}")
            # Replace with a valid action for your environment
            action = 0
            obs, reward, terminated, truncated, info = client.step(action)
            print(f"Obs: {obs}, Reward: {reward}, Terminated: {terminated}, Truncated: {truncated}, Info: {info}")
            if terminated or truncated:
                print("Episode finished, resetting...")
                obs = client.reset()
                print(f"Reset observation: {obs}")
            time.sleep(0.5)
    finally:
        print("Closing client.")
        client.close()
