from assembly import *


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

a = return_prototype()
a.plot_assembly()
