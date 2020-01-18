import json

from assembly import *
import dill
# a = return_prototype()
# print(is_vaild_assembleA(a))

# sample = AssemblyA_Sampler()
# sample.create_assemblyA_database(10,num_of_samples_around=10, debug_mode=True)
# database, curve_databas  = sample.get_database(), sample.get_curve_database()
# print(len(database))
# print(len(curve_databas))
# sample.save(r"C:\Users\A\Desktop\temp")
#
# for i,ass in enumerate(database):
#     ass.plot_assembly(plot_path =r"C:\Users\A\Desktop\temp" , image_number = i,save_images = True)

input_file = open(r"C:\Users\A\Desktop\temp\sampler", 'rb')
sample = dill.load(input_file)

target = create_assemblyA()
print(is_vaild_assembleA(target))
target_curve = get_assembly_curve(target,number_of_points=72)
db_closest_curve, all_dist = sample.get_closest_curve(target_curve, get_all_dis = True)
for k in all_dist:
    print(all_dist[k])
print("-------")
print(all_dist[db_closest_curve])


# input_file = open(r"C:\Users\A\Desktop\temp\sampler", 'rb')
# sample = dill.load(input_file)

# database, curve_databas = create_assemblyA_database(10)
#
# print(len(database))
# print(len(curve_databas))
#
# # config = create_assemblyA().config
#
# config = database[5].config
# print("------------")
# for key1 in config:
#     print(key1)
#     if isinstance(config[key1], dict):
#         for key in config[key1]:
#             #if key not in ["center","orientation","edge"]:
#             print(key)
#             print(config[key1][key])
#     else:
#         print(config[key1])

# a = return_prototype()
# a.plot_assembly()
