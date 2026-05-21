```shell
# build, only if changes in cpp
docker build -f docker/Dockerfile -t rcs-dev .


# run docker

docker compose -f docker/compose/dev.yml run --rm rcs
bash
export PYTHONPATH="/workspace/robot-control-stack/vlagents/src"
python examples/inference/franka.py

```


# act
```shell
python -m vlagents start-server lerobot --port 20000 --host 0.0.0.0 --kwargs '{"policy_name": "act", "checkpoint_path": "/hdd_data/juelg/duobench/weights/act/bin_sort/real/duobench_act_bin_sort_real_2026-05-18_21-54-59/150000/pretrained_model", "n_action_steps": 1}'

python -m vlagents start-server lerobot --port 20000 --host 0.0.0.0 --kwargs '{"policy_name": "act", "checkpoint_path": "/hdd_data/juelg/duobench/weights/act/transfer_cube/duobench_act_transfer_cube_real_2026-05-18_21-55-01/150000/pretrained_model/", "n_action_steps": 1}'
```

# pi0
```shell
python -m vlagents start-server lerobot --port 20000 --host 0.0.0.0 --kwargs '{"policy_name": "pi05", "checkpoint_path": "/hdd_data/juelg/duobench/weights/pi05/duobench_pi05_merged_real_v1_2026-05-19_07-14-26/040000/pretrained_model", "n_action_steps": 1}'

python -m vlagents start-server lerobot --port 20000 --host 0.0.0.0 --kwargs '{"policy_name": "pi05", "checkpoint_path": "/hdd_data/juelg/duobench/weights/pi05/duobench_pi05_bin_sort_real_2026-05-19_22-52-01/030000/pretrained_model", "n_action_steps": 1}'
```



# xvla
```shell
python -m vlagents start-server lerobot --port 20000 --host 0.0.0.0 --kwargs '{"policy_name": "xvla", "checkpoint_path": "/hdd_data/juelg/duobench/weights/xvla/duobench_xvla_merged_real_v1_2026-05-19_17-00-27/040000/pretrained_model", "n_action_steps": 1, "rename_map": {"head": "image", "left_wrist": "image2", "right_wrist": "image3"}}'
```


# tasks
## bin sort
```
0: "pick up a box",
1: "pick up the other box with the other arm",
2: "place the box in the correct bin",
3: "task completed; both boxes are in the correct bins",
```