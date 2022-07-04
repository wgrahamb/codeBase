import copy
import numpy as np
import pandas as pd
import math
import os
import pickle

import Functions as fxn
from Guidance.InterceptorBasic import InterceptorBasicGuidance
from Guidance.ThreatBasic import ThreatBasicGuidance
from ShotPattern.shot_patterns_v2 import PatternShots

class NOVICE:
	def __init__(self):
		self.data = {}
		self.players = {
			"assets": [],
			"launchers": [],
			"soldiers": [],
			"threats": []
		}
		self.simulate = True
		self.simtime = 0.0
		self.timestep = 0.05
		self.sp = None

	@staticmethod
	def asset():
		asset = {
			"ID": None,
			"pos": None,
			"simtime": None,
			"status": None,
		}
		return asset

	@staticmethod
	def interceptor():
		interceptor = {
			"ID": None,
			"launcherID": None,
			"pos": None,
			"vel": None,
			"acc": None,
			"simtime": None
		}
		return interceptor

	@staticmethod
	def interceptortypes():
		types = [
			{
				"skill": np.array([(4.0 * 9.81) / 400.0]),
				"drag": np.array([0.0])
			},
			{
				"skill": np.array([(4.0 * 9.81) / 500.0]),
				"drag": np.array([0.0])
			},
			{
				"skill": np.array([(4.0 * 9.81) / 600.0]),
				"drag": np.array([0.0])
			},
		]
		return types

	@staticmethod
	def launcher():
		launcher = {
			"ID": None,
			"pos": None,
			"inventory": None,
			"simtime": None,
			"interceptortype": None
		}
		return launcher

	@staticmethod
	def soldier():
		soldier = {
			"ID": None,
			"LauncherID": None,
			"status": None,
			"simtime": None,
			"TOF": None,
			"initpos": None,
			"initvel": None,
			"params": None,
			"evaluation": None,
			"projectory": None,
			"ep": None,
			"pip": None,
			"tca": None,
			"ldc": None,
			"interceptor": None,
		}
		return soldier

	@staticmethod
	def threat():
		threat = {
			"ID": None,
			"pos": None,
			"vel": None,
			"acc": None,
			"targetID": None,
			"targetpos": None,
			"simtime": None,
			"TOF": None,
			"TGO": None,
			"params": None,
			"status": None
		}
		return threat

	@staticmethod
	def threattypes():
		types = [
			{
				"maneuveralt": np.array([16000.0]),
				"skill": np.array([(5.0 * 9.81) / 600.0]),
				"drag": np.array([0.1])
			},
			{
				"maneuveralt": np.array([17000.0]),
				"skill": np.array([(5.0 * 9.81) / 700.0]),
				"drag": np.array([0.1])
			},
			{
				"maneuveralt": np.array([18000.0]),
				"skill": np.array([(5.0 * 9.81) / 800.0]),
				"drag": np.array([0.1])
			},
		]
		return types

	def generate_scene(self, nAssets, nInterceptors, nThreats):
		for i in range(nAssets):
			asset = copy.deepcopy(self.asset())
			asset["ID"] = i
			asset["pos"] = np.insert(arr=(np.random.uniform(0, high=[5000, 5000], size=2)), obj=2, values=0)
			asset["simtime"] = self.simtime
			asset["status"] = "ALIVE"
			self.players["assets"].append(asset)
		nLaunchers = math.ceil(nInterceptors / 20.0)
		interceptors = list(range(nInterceptors))
		for i in range(nLaunchers):
			launcher = copy.deepcopy(self.launcher())
			launcher["ID"] = i
			launcher["pos"] = np.insert(arr=(np.random.uniform(0, high=[5000, 5000], size=2)), obj=2, values=0)
			launcher["inventory"] = []
			type = (np.random.randint(0, high=3, size=1))[0]
			launcher["interceptortype"] = type
			ii = -1
			while len(interceptors) > 0:
				ii += 1
				interceptors.pop(0)
				interceptor = copy.deepcopy(self.interceptor())
				interceptor["ID"] = ii + i * 20
				interceptor["pos"] = launcher["pos"]
				interceptor["vel"] = np.zeros(3)
				interceptor["acc"] = np.zeros(3)
				interceptor["simtime"] = self.simtime
				launcher["inventory"].append(interceptor)
				if ii > 18:
					break
			self.players["launchers"].append(launcher)
		for i in range(nThreats):
			threat = copy.deepcopy(self.threat())
			threat["ID"] = i
			threat["pos"] = np.random.uniform(15000, high=[20000, 20000, 20000], size=3)
			type = (np.random.randint(0, high=3, size=1))[0]
			threat["params"] = self.threattypes()[type]
			threat["vel"] = np.random.uniform(-1000, high=[-500, -500, -500], size=3)
			threat["targetID"] = math.floor(np.random.randint(0, high=[len(self.players["assets"])], size=1))
			threat["targetpos"] = self.players["assets"][threat["targetID"]]["pos"]
			threat["TOF"] = self.simtime
			threat["status"] = "ALIVE"
			threat["simtime"] = self.simtime
			self.players["threats"].append(threat)
		self.store_data()
		self.writepickle("SCENE.pickle")

	def load_scene(self, filepath):
		scene = pickle.load(open(r"{}".format(filepath), "rb"))
		self.players = copy.deepcopy(scene["0.0"])

	def store_data(self):
		self.data["{}".format(self.simtime)] = copy.deepcopy(self.players)

	def writepickle(self, filename):
		dirname = os.getcwd()
		with open(r"{}\{}".format(dirname, filename), "wb") as file:
			pickle.dump(self.data, file)

	def update_simtime(self):
		newsimtime = round(self.simtime + self.timestep, 2)
		self.simtime = newsimtime

	def update_player_simtime(self):
		for player in self.players:
			for sub in self.players["{}".format(player)]:
				sub["simtime"] = self.simtime

	def simulation(self):
		print("INITIALIZING")
		self.soldiers_init()
		iteration = -1
		while self.simulate:
			iteration += 1
			print(self.simtime)
			self.update_simtime()
			self.update_player_simtime()
			self.update_threats()
			self.update_soldiers()
			self.store_data()
			if iteration > 200:
				break
		self.writepickle("NOVICE.pickle")

	def update_threats(self):
		for threat in self.players["threats"]:
			print("Threat: ", threat["ID"])
			print("   ", "Position: ", threat["pos"])
			print("   ", "Status: ", threat["status"])
			print("   ", "Speed: ", np.linalg.norm(threat["vel"]))
			if threat["status"] == "ALIVE":
				config = [threat["pos"],
						  threat["vel"],
						  threat["targetpos"],
						  np.zeros(3),
						  threat["params"]["maneuveralt"],
						  threat["params"]["skill"],
						  threat["params"]["drag"],
						  np.array([0.0, 0.0, -0.8]),
						  np.array([1.0, 0.0, 0.0]),
						  np.array([0.0, 1.0, 0.0]),
						  np.array([0.0, 0.0, 1.0]),
						  np.zeros(3)]
				c = fxn.convert_pylist_to_nblist(config)
				g = ThreatBasicGuidance(c)
				data = g.guide()
				nxt = fxn.update(threat["pos"], threat["vel"], data[0], self.timestep)
				threat["pos"] = nxt[0]
				threat["vel"] = nxt[1]
				threat["acc"] = data[0]
				threat["TOF"] = self.simtime
				threat["TGO"] = g.TGO
				if not data[1]:
					threat["status"] = "DEAD"
				print("")

	def update_soldiers(self):
		pass

	def soldiers_init(self):
		evaluation = [
			fxn.projectorize_t("ThreatBasicGuidance",
							   fxn.convert_pylist_to_nblist(
								   [
									   threat["pos"],
									   threat["vel"],
									   asset["pos"],
									   np.zeros(3),
									   threat["params"]["maneuveralt"],
									   threat["params"]["skill"],
									   threat["params"]["drag"],
									   np.array([0.0, 0.0, -0.8]),
									   np.array([1.0, 0.0, 0.0]),
									   np.array([0.0, 1.0, 0.0]),
									   np.array([0.0, 0.0, 1.0]),
									   np.zeros(3)
								   ]
							   ),
							   threat["TOF"])
			if threat["status"] == "ALIVE" else None for threat in self.players["threats"]
			for asset in self.players["assets"]
		]
		i = 0
		for e in evaluation:
			for launcher in self.players["launchers"]:
				soldier = copy.deepcopy(self.soldier())
				soldier["LauncherID"] = launcher["ID"]
				soldier["status"] = "INACTIVE"
				soldier["simtime"] = self.simtime
				soldier["TOF"] = 0.0
				soldier["initpos"] = launcher["pos"]
				soldier["params"] = self.interceptortypes()[launcher["interceptortype"]]
				soldier["evaluation"] = [pd.DataFrame(data=e[0],
													  columns=["x", "y", "z", "vx", "vy", "vz", "TOF"])]
				threat_trajectory = soldier["evaluation"][0].to_numpy()
				print("")
				firstshot = fxn.find_firstshot(soldier["initpos"],
											   np.array([300.0, 300.0, 500.0]),
											   soldier["params"]["skill"],
											   soldier["params"]["drag"],
											   soldier["TOF"],
											   threat_trajectory)
				firsttof = np.array([firstshot[-1][-1]])
				bestshot = fxn.find_bestshot(soldier["initpos"],
											 np.array([300.0, 300.0, 500.0]),
											 soldier["params"]["skill"],
											 soldier["params"]["drag"],
											 soldier["TOF"],
											 firsttof,
											 threat_trajectory)
				if not bestshot[2]:
					bestshot = firstshot
				soldier["initvel"] = np.array([bestshot[0][0][3],
											   bestshot[0][0][4],
											   bestshot[0][0][5]])
				soldier["projectory"] = [pd.DataFrame(data=bestshot[0],
													  columns=["x", "y", "z", "vx", "vy", "vz", "TOF"])]
				dp1 = np.delete(bestshot[1], [-1, -2, -3])
				dp2 = np.delete(bestshot[0][-1], -1)
				ep = fxn.closing_velocities_ep(dp1, dp2)
				soldier["ep"] = ep
				soldier["pip"] = bestshot[1]
				tman_alt = np.array(e[1][4])
				tskill = np.array(e[1][5])
				tdrag = np.array(e[1][6])
				threat_capability = self.tcacalc(soldier, tman_alt, tskill, tdrag)
				soldier["evaluation"] = soldier["evaluation"] + threat_capability[0]
				soldier["tca"] = [threat_capability[1]]
				soldier["ldc"] = []
				interceptor_capability = self.ldccalc(soldier)
				soldier["projectory"] = soldier["projectory"] + interceptor_capability[0]
				soldier["ldc"] = [interceptor_capability[1]]
				# self.sp = PatternShots
				# shotplan = self.sp(soldier["tca"][0], soldier["ldc"][0])
				# self.replan()
				self.players["soldiers"].append(soldier)

				# idindex = -1
				# soldier["ldc"] = []
				# for ldc in shotplan.df_LDCs:
				#     idindex += 1
				#     temp = []
				#     for index, row in ldc.iterrows():
				#         eppoint = np.array([row["EPx"], row["EPy"], 0.0])
				#         enupoint = fxn.rotate_into_enu(np.array([-eppoint[0], -eppoint[1], 0.0]),
				#                                        np.array([ep[0], ep[1], ep[2], ep_o]))
				#         temp.append([idindex,
				#                      enupoint[0], enupoint[1], enupoint[2],
				#                      eppoint[0], eppoint[1], eppoint[2],
				#                      int(0)])
				#     newldc = pd.DataFrame(data=temp,
				#                           columns=["ID", "x", "y", "z", "EPx", "EPy", "EPz", "TCA_ID"])
				#     soldier["ldc"].append(newldc)
				# self.players["soldiers"].append(soldier)

	@staticmethod
	def tcacalc(soldier, tman_alt, tskill, tdrag):
		threatstart = soldier["evaluation"][0].iloc[0]
		threatend = soldier["evaluation"][0].iloc[-1]
		soldierend = soldier["projectory"][0].iloc[-1]
		ep = soldier["ep"]
		ep_o = np.array([soldierend["x"],
						 soldierend["y"],
						 soldierend["z"]])
		pos1 = np.array([threatstart["x"], threatstart["y"], threatstart["z"]])
		vel1 = np.array([threatstart["vx"], threatstart["vy"], threatstart["vz"]])
		tpos = np.array([threatend["x"], threatend["y"], threatend["z"]])
		tvel = np.zeros(3)
		ufd = fxn.los_ufd(pos1, tpos)
		tca = []
		shots = []
		for direction in ["UP", "RIGHT", "DOWN", "LEFT"]:
			for_eval = []
			for spacer in np.linspace(0.1, 1.0, 10):
				if direction == "UP":
					fdir = np.array([ufd[0], ufd[1], ufd[2] - spacer])
				elif direction == "RIGHT":
					fdir = np.array([ufd[0] - spacer,
									 ufd[1] + spacer,
									 ufd[2] - 0.4])
				elif direction == "DOWN":
					fdir = np.array([ufd[0], ufd[1], ufd[2] + spacer])
				elif direction == "LEFT":
					fdir = np.array([ufd[0] + spacer,
									 ufd[1] - spacer,
									 ufd[2] - 0.4])
				config1 = [pos1,
						   vel1,
						   tpos,
						   tvel,
						   tman_alt,
						   tskill,
						   tdrag,
						   fdir,
						   ep[0],
						   ep[1],
						   ep[2],
						   ep_o]
				c1 = fxn.convert_pylist_to_nblist(config1)
				shotdata1 = fxn.projectorize_t("ThreatBasicGuidance",
											   c1,
											   threatstart["TOF"])
				shot1 = pd.DataFrame(data=shotdata1[0],
									 columns=["x", "y", "z", "vx", "vy", "vz", "TOF"])
				pos2 = np.array([shot1.iloc[-1]["x"],
								 shot1.iloc[-1]["y"],
								 shot1.iloc[-1]["z"]])
				vel2 = np.array([shot1.iloc[-1]["vx"],
								 shot1.iloc[-1]["vy"],
								 shot1.iloc[-1]["vz"]])
				a = np.array([shot1.iloc[-2]["x"],
							  shot1.iloc[-2]["y"],
							  shot1.iloc[-2]["z"]])
				epp1 = fxn.rotate_into_ep(pos2,
										  np.array([ep[0], ep[1], ep[2], ep_o]))
				epp2 = fxn.rotate_into_ep(a,
										  np.array([ep[0], ep[1], ep[2], ep_o]))
				tcapointep = fxn.parametrize(epp1, epp2)
				tcapointenu = fxn.rotate_into_enu(np.array([-tcapointep[0], -tcapointep[1], tcapointep[2]]),
												  np.array([ep[0], ep[1], ep[2], ep_o]))
				tcapoint = [0, tcapointenu[0], tcapointenu[1], tcapointenu[2],
							tcapointep[0], tcapointep[1], tcapointep[2]]
				config2 = [pos2,
						   vel2,
						   tpos,
						   tvel,
						   tman_alt,
						   tskill,
						   tdrag,
						   fdir,
						   np.array([1.0, 0.0, 0.0]),
						   np.array([0.0, 1.0, 0.0]),
						   np.array([0.0, 0.0, 1.0]),
						   np.zeros(3)]
				c2 = fxn.convert_pylist_to_nblist(config2)
				shotdata2 = fxn.projectorize_t("ThreatBasicGuidance",
											   c2,
											   shot1.iloc[-1]["TOF"])
				shot2 = pd.DataFrame(data=shotdata2[0],
									 columns=["x", "y", "z", "vx", "vy", "vz", "TOF"])
				shot = pd.concat([shot1, shot2])
				for_eval.append([shot, tcapoint])
			index = -1
			next_for_eval = []
			for e in for_eval:
				index += 1
				dp = e[0].iloc[-1]
				evalend = np.array([dp["x"], dp["y"], dp["z"]])
				dist = fxn.distance(tpos, evalend)
				if dist < 100.0:
					if direction == "UP":
						next_for_eval.append([e[1][5], index])
					elif direction == "RIGHT":
						next_for_eval.append([e[1][4], index])
					elif direction == "DOWN":
						next_for_eval.append([e[1][5], index])
					elif direction == "LEFT":
						next_for_eval.append([e[1][4], index])
			if direction == "UP":
				df = pd.DataFrame(data=next_for_eval,
								  columns=["metric", "index"])
				m = df.loc[df["metric"] == df["metric"].min()]
			elif direction == "RIGHT":
				df = pd.DataFrame(data=next_for_eval,
								  columns=["metric", "index"])
				m = df.loc[df["metric"] == df["metric"].max()]
			elif direction == "DOWN":
				df = pd.DataFrame(data=next_for_eval,
								  columns=["metric", "index"])
				m = df.loc[df["metric"] == df["metric"].max()]
			elif direction == "LEFT":
				df = pd.DataFrame(data=next_for_eval,
								  columns=["metric", "index"])
				m = df.loc[df["metric"] == df["metric"].min()]
			index = int(m.iloc[0]["index"])
			shots.append(for_eval[index][0])
			tca.append(for_eval[index][1])
		tca.append(tca[0])
		tca = pd.DataFrame(data=tca,
						   columns=["ID", "x", "y", "z", "EPx", "EPy", "EPz"])
		return shots, tca

	@staticmethod
	def ldccalc(soldier):
		trajectory = soldier["evaluation"][0].to_numpy()
		UP = np.array([0.0, 1.0, 0.0])
		RIGHT = np.array([-1.0, 0.0, 0.0])
		DOWN = np.array([0.0, -1.0, 0.0])
		LEFT = np.array([1.0, 0.0, 0.0])
		soldierfinalstate = soldier["projectory"][0].iloc[-1]
		ep_o = np.array([soldierfinalstate["x"],
						 soldierfinalstate["y"],
						 soldierfinalstate["z"]])
		ep_input = np.array([soldier["ep"][0],
							 soldier["ep"][1],
							 soldier["ep"][2],
							 ep_o])
		fiftypercenttof = round(soldier["projectory"][0].iloc[-1]["TOF"] / 2, 2)
		interceptorstartindex = soldier["projectory"][0].loc[soldier["projectory"][0]["TOF"] == fiftypercenttof]
		interceptorstart = soldier["projectory"][0].iloc[interceptorstartindex.index[0]]
		pos = np.array([interceptorstart["x"], interceptorstart["y"], interceptorstart["z"]])
		vel = np.array([interceptorstart["vx"], interceptorstart["vy"], interceptorstart["vz"]])
		skill = soldier["params"]["skill"]
		drag = soldier["params"]["drag"]
		soldier_tof = interceptorstart["TOF"]
		tof1 = trajectory[0][-1]
		tof2 = soldier["pip"][-1]
		shots = []
		for fdir in [UP, RIGHT, DOWN, LEFT]:
			data = fxn.find_containment_limit(trajectory,
											  fdir,
											  ep_input,
											  pos,
											  vel,
											  skill,
											  drag,
											  soldier_tof,
											  tof1,
											  tof2)
			# if data[2]:
			#     shots.append(data)
			# else:
			#     shots.append([soldier["projectory"][0].to_numpy()])
			shots.append(data)
		ldc = []
		ret_shots = []
		for shot in shots:
			ret_shots.append(
				pd.DataFrame(data=shot[0],
							 columns=["x", "y", "z", "vx", "vy", "vz", "TOF"])
			)
			data = shot[0][-1]
			ldcpos = np.array([data[0], data[1], data[2]])
			ldcposep = fxn.rotate_into_ep(ldcpos,
										  ep_input)
			ldcposenu = fxn.rotate_into_enu(-ldcposep,
											ep_input)
			ldc.append([0, ldcposenu[0], ldcposenu[1], ldcposenu[2],
						ldcposep[0], ldcposep[1], ldcposep[2], 0])
		ldc.append(ldc[0])
		ldc = pd.DataFrame(data=ldc,
						   columns=["ID", "x", "y", "z", "EPx", "EPy", "EPz", "TCA_ID"])
		return ret_shots, ldc

	def replan(self):
		pass



if __name__ == "__main__":
	sim = NOVICE()
	sim.generate_scene(nAssets=1, nInterceptors=20, nThreats=1)
	# sim.load_scene(r"C:\Users\wbeech\Graham\pythonrepo\CONTAINMENT\NOVICE4\SCENE.pickle")
	sim.simulation()
