const Point3DS{T} = Point3D{SVector{3, T}}

zero_nans(x) = ifelse(isnan(x), zero(x), x)

function canonicalize_body_fixed_points!(d::Associative{<:RigidBody, <:AbstractVector{<:Point3D}})
    for (body, points) in d
        for i in eachindex(points)
            point = points[i]
            if point.frame != default_frame(body)
                point = frame_definition(body, point.frame) * point
                points[i] = point
            end
        end
    end
end
