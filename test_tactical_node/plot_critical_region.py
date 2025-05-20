import tactical_node.critical_region as critical_region


def main():

    EGO_PATH_START_X = 5
    EGO_PATH_START_Y = 0
    EGO_PATH_END_X = -5
    EGO_PATH_END_Y = 0
    EGO_PATH_START =   (EGO_PATH_START_X, EGO_PATH_START_Y)
    EGO_PATH_END   =   (EGO_PATH_END_X, EGO_PATH_END_Y)

    ADV_PATH_START_X = 0
    ADV_PATH_START_Y = -5
    ADV_PATH_END_X =   0
    ADV_PATH_END_Y =   5
    ADV_PATH_START =   (ADV_PATH_START_X, ADV_PATH_START_Y)
    ADV_PATH_END   =   (ADV_PATH_END_X, ADV_PATH_END_Y)

    CR_POINT_1 = (0.5, 0.5)
    CR_POINT_2 = (0.5, -0.5)
    CR_POINT_3 = (-0.5, -0.5)
    CR_POINT_4 = (-0.5, 0.5)

    critical_region_poly = critical_region.create_cr_polygon([CR_POINT_1,
                                                                CR_POINT_2,
                                                                CR_POINT_3,
                                                                CR_POINT_4])

    target_path = critical_region.create_path(ADV_PATH_START, ADV_PATH_END)
    target_cr = critical_region.CriticalRegion(target_path, critical_region_poly)
    target_cr.compute_critical_points()

    ego_path = critical_region.create_path(EGO_PATH_START, EGO_PATH_END)
    ego_cr = critical_region.CriticalRegion(ego_path, critical_region_poly)
    ego_cr.compute_critical_points()

    target_cr.plot_regions("TARGET")
    ego_cr.plot_regions("EGO")


if __name__ == '__main__':

    main()