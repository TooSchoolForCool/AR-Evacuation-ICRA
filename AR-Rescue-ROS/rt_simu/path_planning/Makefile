.PHONY: sample simu

MAP_FILE = "assets/map.png"
N_AGENTS = 100


sample:
	python agent_sampling.py $(MAP_FILE) $(N_AGENTS) "assets/agents.nparray"


simu:
	python simu.py $(MAP_FILE)


eval_time:
	python eval_time.py $(MAP_FILE)
