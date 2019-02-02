function deconstruct(ordered_marker_bodies::AbstractVector{<:RigidBody}, q::AbstractVector,
        marker_positions_body::AbstractDict{<:RigidBody, <:AbstractVector{<:Point3D}})
    x = Vector(q)
    for body in ordered_marker_bodies
        positions = marker_positions_body[body]
        for j = 1 : length(positions)
            append!(x, positions[j].v)
        end
    end
    x
end

function reconstruct!(ordered_marker_bodies::AbstractVector{<:RigidBody}, q::AbstractVector,
        marker_positions_body::AbstractDict{<:RigidBody, <:AbstractVector{<:Point3D}}, x...)
    index = 1
    for i = 1 : length(q)
        q[i] = x[index]
        index += 1
    end
    for body in ordered_marker_bodies
        positions = marker_positions_body[body]
        frame = default_frame(body)
        for j = 1 : length(positions)
            positions[j] = Point3D(frame, x[index], x[index + 1], x[index + 2])
            index += 3
        end
    end
end
