@require RigidBodyTreeInspector begin
    using RigidBodyTreeInspector
    using DrakeVisualizer
    using Interact
    import ColorTypes: RGBA
    import GeometryTypes: HyperSphere, Point

    function RigidBodyTreeInspector.inspect!(state::MechanismState, vis::Visualizer, problem::CalibrationProblem, result::CalibrationResult)
        radius = 0.01
        semitransparent_green = RGBA(0., 1, 0, 0.5)
        for points in values(result.marker_positions)
            for point in points
                sphere = HyperSphere(Point{3, Float64}(point.v), radius)
                addgeometry!(vis, state.mechanism, point.frame, GeometryData(sphere, semitransparent_green))
            end
        end

        cal_slider = selection_slider(["Before cal" => false, "After cal" => true])
        pose_slider = slider(1 : num_poses(result), value = 1, label = "Pose number")
        map(signal(pose_slider), signal(cal_slider)) do i, cal
            # TODO: show measurements
            set_configuration!(state, result.configurations[i])
            if !cal
                q_before_cal = problem.pose_data[i].configuration
                for joint in keys(problem.calibration_param_bounds)
                    set_configuration!(state, joint, q_before_cal[configuration_range(state, joint)])
                end
            end
            settransform!(vis, state)
        end

        display.([cal_slider; pose_slider])
        nothing
    end
end
