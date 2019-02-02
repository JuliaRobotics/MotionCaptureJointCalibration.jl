const Point3DS{T} = Point3D{SVector{3, T}}

zero_nans(x) = ifelse(isnan(x), zero(x), x)

function canonicalize(body::RigidBody, point::Point3D)
    if point.frame != default_frame(body)
        point = frame_definition(body, point.frame) * point
    end
    point
end

function canonicalize!(d::Dict{RigidBody{T}, Vector{Tuple{Point3DS{T}, Point3DS{T}}}}) where {T}
    for (body, bounds) in d
        for i in eachindex(bounds)
            bounds[i] = canonicalize.(Ref(body), bounds[i])
        end
    end
end

const TreeJointSegmentedVector{T} = SegmentedVector{
        RigidBodyDynamics.JointID, T, Base.OneTo{RigidBodyDynamics.JointID}, Vector{T}}
