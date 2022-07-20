# -*- coding: utf-8 -*-
"""
Created on Wed May 19 10:59:33 2021

@author: msommers
"""

import os
import pickle
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import subprocess

from importlib_metadata import version



#=============================================================================
# Reformatting 
#=============================================================================

# For Graham's OLD output
def reformat_data(_dict):
    # Only useful for reformatting specific output
    new_dict = {}
    point_keys = [k for k in list(_dict.keys()) if "poly_point" in k]
    num_points = int(len(point_keys)/3)  #x,y,z for each
    for idx, t in enumerate(_dict["simtime"]):
        if idx <= len(_dict["poly_point_x1"])-1:
            new_dict[t] = {}
            df = pd.DataFrame(columns=["x","y","z"])
            counter = 0
            for pt in range(num_points):
                keys = [k for k in point_keys if str(pt+1) in k]
                if keys:
                    kx, ky, kz = keys[0], keys[1], keys[2]
                    df.loc[counter, "x"] = float(_dict[kx][idx])
                    df.loc[counter, "y"] = float(_dict[ky][idx])
                    df.loc[counter, "z"] = float(_dict[kz][idx])
                    counter+=1
        else:
            break
        new_dict[t]["TDC_points"] = df
    
    return new_dict


# For Graham's NEW output
def data_reformatter(_dict):
    # Only useful for reformatting specific output
    new_dict = {}
    times = sorted(list(_dict.keys()))
    for t in times:
        tca_df = _dict[t]["tca_points"]["ep_points"]
        tca_enu_df = _dict[t]["tca_points"]["enu_points"]
        if len(tca_df) > 2:  # 3 points required for polygon
            new_dict[t] = {"TCA_points": tca_df,
                           "ENU_points": tca_enu_df}
            
    return new_dict



# For Graham's NEW output
def data_reformatter_v2(_dict):
    # Only useful for reformatting specific output
    new_dict = {}
    times = sorted(list(_dict.keys()))
    for t in times:
        tca_dict = _dict[t]["tca_points"]
        tca_ep = tca_dict["ep_points"]
        tca_enu = tca_dict["enu_points"]
        if len(tca_ep) > 2:  # 3 points required for polygon
            new_dict[t] = {"TCA_EP_points": tca_ep,
                           "TCA_ENU_points": tca_enu}
    return new_dict


def data_reformatter_v3(_dict):
    # Only useful for reformatting specific output
    new_dict = {}
    times = sorted(list(_dict.keys()))
    for t in times:
        new_dict[t] = {}
        # print("Time {}".format(t))
        assets = _dict[t]["assets"]
        interceptors = _dict[t]["interceptors"]
        launchers = _dict[t]["launchers"]
        shooters = _dict[t]["shooters"]
        threats = _dict[t]["threats"]
        
        new_dict[t]["TCA_EP_points"] = []
        new_dict[t]["TCA_ENU_points"] = []
        new_dict[t]["LDC_EP_points"] = []
        new_dict[t]["LDC_ENU_points"] = []
        
        num = 0
        for thr in threats:
            global tcas, ldcs
            tcas = thr["tcas"] 
            # thr_id = thr["ID"]
            for tca in tcas:
                tca.rename(columns={"x":"ENUx", "y":"ENUy", "z":"ENUz",
                                    "vx":"ENUvx", "vy":"ENUvy", "vz":"ENUvz",
                                    "x_ep":"EPx", "y_ep":"EPy", "z_ep":"EPz"},
                           inplace=True)
                tca.loc[:, "Time"] = t
                tca.loc[:, "ID"] = num
                enu_df = tca[["Time", "ID", "ENUx","ENUy","ENUz", "TOF"]]
                ep_df = tca[["Time", "ID", "EPx","EPy","EPz"]]
                
                new_dict[t]["TCA_EP_points"].append(ep_df)
                new_dict[t]["TCA_ENU_points"].append(enu_df)
                num += 1
            # print()
        # num = 0
        for sh in shooters:
            ldcs = sh["ldcs"]
            tca_idx = sh["tcaindex"]
            for ldc in ldcs:    # there should only ever be 1!
                ldc.rename(columns={"x":"ENUx", "y":"ENUy", "z":"ENUz",
                                    "vx":"ENUvx", "vy":"ENUvy", "vz":"ENUvz",
                                    "x_ep":"EPx", "y_ep":"EPy", "z_ep":"EPz"},
                           inplace=True)
                ldc.loc[:, "Time"] = t
                ldc.loc[:, "ID"] = tca_idx  #num
                enu_df = ldc[["Time", "ID", "ENUx","ENUy","ENUz", "TOF"]]
                ep_df = ldc[["Time", "ID", "EPx","EPy","EPz"]]
                new_dict[t]["LDC_EP_points"].append(ep_df)
                new_dict[t]["LDC_ENU_points"].append(enu_df)
                # num += 1
            # print()
        # print()
    return new_dict

    


def data_reformatter_v4(_dict):
    new_dict = {}
    times = sorted(list(_dict.keys()))
    for t in times:
        new_dict[t] = {}
        assets = _dict[t]["assets"]
        interceptors = _dict[t]["interceptors"]
        launchers = _dict[t]["launchers"]
        shooters = _dict[t]["shooters"]
        threats = _dict[t]["threats"]
        
        # new_dict[t]["TCA"] = []
        tca_concat = []
        tca_cols = ["Time", "ID", "x", "y", "z", "EPx", "EPy", "EPz"]
        
        # new_dict[t]["LDC"] = []
        ldc_concat = []
        ldc_cols = tca_cols + ["TCA_ID"]

        num = 0
        for thr in threats:
            tcas = thr["tcas"]    
            for tca in tcas:
                tca.rename(columns={"x_ep":"EPx", "y_ep":"EPy", "z_ep":"EPz"},
                           inplace=True)
                tca.loc[:, "Time"] = t
                tca.loc[:, "ID"] = num
                # new_dict[t]["TCA"].append(tca[tca_cols])
                tca_concat.append(tca[tca_cols])        
                num += 1
        if tca_concat:
            new_dict[t]["TCA"] = pd.concat(tca_concat)
                
        num = 0
        for sh in shooters:
            ldcs = sh["ldcs"]
            tca_idx = sh["tcaindex"]
            threat_id = sh["targetID"]
            threat = threats[threat_id]
            
            for ldc in ldcs:
                ldc.rename(columns={"x_ep":"EPx", "y_ep":"EPy", "z_ep":"EPz"},
                           inplace=True)
                ldc.loc[:, "Time"] = t
                ldc.loc[:, "ID"] = num
                ldc.loc[:, "TCA_ID"] = threat["tcas"][tca_idx]["ID"].iloc[0]
                # new_dict[t]["LDC"].append(ldc[ldc_cols])
                ldc_concat.append(ldc[ldc_cols])
                num += 1
        if ldc_concat:
            new_dict[t]["LDC"] = pd.concat(ldc_concat)
        
    return new_dict



def format_nsim_data(scenario_name):
    nsim_dir = r"C:\Users\msommers\Desktop\NSim\Full20210607CBFC\Full20210607\NSim"
    bin_dir = os.path.join(nsim_dir, "bin") 
    
    scenario_dir = os.path.join(bin_dir, scenario_name)
    data_dir = scenario_dir     # directory to read output from (bin_dir)
    
    tca_file = os.path.join(data_dir, "TCAPatch.txt")
    ldc_file = os.path.join(data_dir, "LDCPatch.txt")
    
    header = "Time,ID,Type,ENUx,ENUy,ENUz,EPx,EPy,Zero".split(",")
    tca_df = pd.read_csv(tca_file, delimiter=" ") 
    ldc_df = pd.read_csv(ldc_file, delimiter=" ")
    
    ldc_df.columns = header
    tca_df.columns = header

    # plt.close("all")
    data_dict = {}
    for t, ldf in ldc_df.groupby("Time"):
        data_dict[t] = {}
        data_dict[t]["LDC_EP_points"] = []
        data_dict[t]["TCA_EP_points"] = []
        
        data_dict[t]["LDC_ENU_points"] = []
        data_dict[t]["TCA_ENU_points"] = []
        
        # Filtering TCA data by ID
        time_filt = tca_df["Time"] == t
        
        ep_cols = ["Time", "ID", "EPx", "EPy"]
        enu_cols = ["Time", "ID", "ENUx", "ENUy", "ENUz"]
        for num, (id_, ldf_) in enumerate(ldf.groupby("ID")):
            
            # Filtering TCA data (by time and ID)
            id_filt = tca_df["ID"] == id_
            filts = time_filt & id_filt
            
            # Actual LDCs and TCAs
            # data_dict[t]["TCA_EP_points"].append(tca_df[filts][ep_cols])
            # data_dict[t]["TCA_ENU_points"].append(tca_df[filts][enu_cols])             
            
            # Switching the LDCs and TCAs for testing purposes only
            data_dict[t]["TCA_EP_points"].append(ldf_[ep_cols])
            data_dict[t]["TCA_ENU_points"].append(ldf_[enu_cols]) 

            # Actual LDCs and TCAs
            # data_dict[t]["LDC_EP_points"].append(ldf_[ep_cols]) 
            # data_dict[t]["LDC_ENU_points"].append(ldf_[enu_cols]) 

            # Switching the LDCs and TCAs for testing purposes only
            data_dict[t]["LDC_EP_points"].append(tca_df[filts][ep_cols]) 
            data_dict[t]["LDC_ENU_points"].append(tca_df[filts][enu_cols]) 
    return data_dict


#=============================================================================
# Requirements File
#=============================================================================
    
# requires the installation of pipreqs
def create_requirements_txt(project_dir):
    file = os.path.join(project_dir, "requirements.txt")
    print("Writing: {}".format(file))
    cmd = 'pipreqs "{}" --force --debug'.format(project_dir)
    completed_process = subprocess.run(cmd)
    return completed_process




#=============================================================================
# 3D Plane 
#=============================================================================

def get_plane_equation(p1, p2, p3):
    # These two vectors are in the plane
    v1 = np.array(p3 - p1, dtype=np.float64)
    v2 = np.array(p2 - p1, dtype=np.float64)
    
    # the cross product is a vector normal to the plane
    cp = np.cross(v1, v2)
    a, b, c = cp
    
    # This evaluates a * x3 + b * y3 + c * z3 which equals d
    d = np.dot(cp, p3)
    
    # Ax + By + Cz = D
    # print('The equation is {0}x + {1}y + {2}z = {3}'.format(a, b, c, d))
    
    # Ax + By + Cz + D = 0
    # print('The equation is {0}x + {1}y + {2}z + {3} = 0'.format(a, b, c, -d))    
    # return a, b, c, -d
    coefficients = (a, b, c, -d)
    return coefficients



def get_dist_from_plane(coefficients, pn):
    # coefficients should be of form (a, b, c, d)
    # Get dist of pn from plane ax + by + cz + d = 0
    # Should be 0 if coplanar
    (a, b, c, d) = coefficients
    x, y, z = pn
    dist = np.abs( a*x + b*y + c*z + d)/np.sqrt(a**2 + b**2 + c**2)
    return dist


def is_coplanar(p1,p2,p3,pn):
    # Determines if pn is coplanar to points p1,p2,p3
    coefficients = get_plane_equation(p1, p2, p3)
    dist = get_dist_from_plane(coefficients, pn)
    # Only truly coplanar if dist = 0
    # Declares coplanar if it's close enough (within arbitrary distance)
    if dist <= 10.0:
        return True, dist
    else:
        return False, dist 



#=============================================================================
# Subplots
#=============================================================================

def calc_rows_cols(num):
    # need a better way to determine a good number of rows and columns for 
    # subplots
    if num == 1:
        rows = 1
        cols = 1
    elif num == 2:
        rows = 1
        cols = 2
    elif num == 3:
        rows = 1 
        cols = 3
    elif num == 4:
        rows = 2 
        cols = 2
    elif num == 5 or num == 6:
        rows = 2
        cols = 3
    elif num == 7 or num == 8:
        rows = 2
        cols = 4
    elif num == 9:
        rows = 3
        cols = 3
    elif num == 10:
        rows = 2
        cols = 5
    elif num == 11 or num == 12:
        rows = 3
        cols = 4
    elif num <=16:
        rows = 4
        cols = 4
    else:
        raise Exception("I haven't figured out what geoms to do yet...")
    return rows, cols















#=============================================================================
# Pickles
#=============================================================================

# ===== Figures =====
def pickle_figure(fig, outdir, filename):
    filepath = os.path.join(outdir, filename)
    name, ext = os.path.splitext(filepath)  
    filepath = name + ".fig.pickle"
    print("Writing: {}".format(filepath))
    pickle.dump(fig, open(filepath, "wb"))
    return filepath


def load_pickle_fig2D(pickle_file):
    with open(pickle_file, "rb") as f:
        fig = pickle.load(f)
    plt.show()
    return fig


def load_pickle_fig3D(pickle_file):
    plt.ioff()
    with open(pickle_file, "rb") as f:
        fig = pickle.load(f)
        plt.close(fig)
        new_fig = recreate_fig(fig)
    plt.show()
    return new_fig


def recreate_fig(fig):
    # Recreating Figure Layout
    size = fig.get_size_inches()
    new_fig = plt.figure(figsize=size)
    for ax in fig.axes:
        proj = ax.name
        # if hasattr(ax, "get_geometry"):   # for figs with Slider axis
        (r, c, num) = ax.get_geometry()
        new_ax = new_fig.add_subplot(r,c, num, projection=proj)
        xlim, ylim = ax.get_xlim(), ax.get_ylim()
        if proj == "3d":
            zlim = ax.get_zlim()
            new_ax.set_zlim(zlim)        
        new_ax.set_xlim(xlim)
        new_ax.set_ylim(ylim)
        new_ax.set_title(ax.get_title())
        
    # Recreating artists
    for ax, new_ax in zip(fig.axes, new_fig.axes):
        for patch in ax.patches: 
            patch.remove()
            patch.set_transform(new_ax.transData)
            new_ax.add_patch(patch)        
        for line in ax.lines:
            line.remove()
            line.set_transform(new_ax.transData)
            new_ax.add_line(line)
        for coll in ax.collections:
            coll.remove()
            coll.set_transform(new_ax.transData)
            new_ax.add_collection(coll)
        if ax.get_legend():
            new_ax.legend()
    
    return new_fig
    

    
# ===== Data =====
def read_pickle_data(file):
    if not os.path.exists(file):
        raise FileNotFoundError("{} does not exist.".format(file))
    else:
        try:
            with open(file, "rb") as f:
                data = pickle.load(f)
        except:
            raise Exception("Unable to read {}".format(file))
    return data




if __name__ == "__main__":
    
    
    
    # Format NSim data
    # scenario_name = "CB_Raid"    # CB_L1, CB_L2, CB_L3, CB_L4x2, CB_Raid, CB_L1_Testing   
    # data_dict = format_nsim_data(scenario_name)
 
    
    # pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\threat_model_data3.pickle"
    # raw_data = read_pickle_data(pickle_file)
    # data = data_reformatter_v2(raw_data)
    
    
    pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\5A_80I_4T.pickle"
    pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\1A_5I_1T.pickle"
    pickle_file = r"C:\Users\msommers\Desktop\Output\threat_model\2A_25I_2T.pickle"
    raw_data = read_pickle_data(pickle_file)
    data = data_reformatter_v4(raw_data)
    


