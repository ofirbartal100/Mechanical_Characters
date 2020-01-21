from assembly import *
import dill

class AssemblyA_Sampler:
    def __init__(self, number_of_points=72, num_of_samples_around=10):
        self.database = []
        self.curve_database = []
        self.number_of_points = number_of_points
        self.num_of_samples_around = num_of_samples_around

    def recursive_sample_assemblyA(self, assemblyA, num_of_samples_around=None, debug_mode=False,second_type = False):
        if not num_of_samples_around:
            num_of_samples_around = self.num_of_samples_around
        accepted_assemblies = []
        for i in range(num_of_samples_around):
            new_assemblyA = sample_from_cur_assemblyA(assemblyA, random_sample=0.5,second_type = second_type)
            if is_vaild_assembleA(new_assemblyA, debug_mode=debug_mode):
                if debug_mode:
                    print("valid assembly!")
                assembly_curve = get_assembly_curve(new_assemblyA, number_of_points=self.number_of_points,
                                                    normelaize_curve=True)
                if is_dissimilar(assembly_curve, self.curve_database):
                    if debug_mode:
                        print(f"----------------added assembly {len(self.curve_database)}----------------")
                    self.curve_database.append(assembly_curve)
                    accepted_assemblies.append(new_assemblyA)
                elif (debug_mode):
                    print(f"assembly too similar to db")

        return accepted_assemblies

    def get_origin_assembly(self,second_type=False):
        origin_assembly = create_assemblyA(second_type=second_type)
        origin_curve = get_assembly_curve(origin_assembly, number_of_points=self.number_of_points,
                                          normelaize_curve=True)

        while not is_dissimilar(origin_curve, self.curve_database):
            origin_assembly = create_assemblyA(second_type=second_type)
            origin_curve = get_assembly_curve(origin_assembly, number_of_points=self.number_of_points,
                                              normelaize_curve=True)

        self.database += [origin_assembly]
        self.curve_database += [origin_curve]

        return origin_assembly, origin_curve

    def create_assemblyA_database(self, min_samples_number=1000, num_of_samples_around=None, debug_mode=False, second_type=False):
        if not num_of_samples_around:
            num_of_samples_around = self.num_of_samples_around
        cur_database_len = len(self.database)

        origin_assembly, _ = self.get_origin_assembly(second_type=second_type)

        if debug_mode:
            print(f"origin_assembly initiaized")
            print(f"curve_database is {self.curve_database}")

        while len(self.database) - cur_database_len < min_samples_number:
            if debug_mode:
                print(f"current database size {len(self.database)}")
            accepted_assemblies = self.recursive_sample_assemblyA(origin_assembly,
                                                                  num_of_samples_around=num_of_samples_around,
                                                                  debug_mode=debug_mode,second_type = second_type)
            self.database += accepted_assemblies
            while len(accepted_assemblies) > 0 and len(self.database) < min_samples_number:
                origin_assembly = accepted_assemblies[0]
                neighbor_accepted_assemblies = self.recursive_sample_assemblyA(origin_assembly,
                                                                               num_of_samples_around=num_of_samples_around,
                                                                               debug_mode=debug_mode,second_type = second_type)
                self.database += neighbor_accepted_assemblies
                accepted_assemblies = accepted_assemblies[1:]
                accepted_assemblies += neighbor_accepted_assemblies
            if debug_mode:
                print("---we will get another origin assembly---")
            origin_assembly, _ = self.get_origin_assembly()

    def get_database(self):
        return self.database

    def get_curve_database(self):
        return self.curve_database

    def get_closest_curve(self, curve, get_all_dis = False):

        min_dis = curve.normA(curve, self.curve_database[0])
        min_curve = self.curve_database[0]
        closest_assembly = self.database[0]
        if get_all_dis:
            all_dist = {}
        for i,db_curve in enumerate(self.curve_database[1:]):
            cur_dis = curve.normA(curve, db_curve)
            if get_all_dis:
                all_dist[db_curve] = cur_dis
            if cur_dis < min_dis:
                closest_assembly = self.database[i+1]
                min_dis = cur_dis
                min_curve = db_curve

        return min_curve,closest_assembly,all_dist if get_all_dis else None

    def save(self, path=r"C:\Users\A\Desktop\temp"):
        with open(path + rf"\sampler.dill", "wb") as handle:
            dill.dump(self, handle)

