function val = wallSurfaceEval(z_query, y_query, params)
    

    wallSurfaceFcn = @(z_query, y_query) interp2(params.mesh_z, params.mesh_y,  params.mesh_x, z_query, y_query, 'linear', 0);
    val = wallSurfaceFcn(z_query, y_query);
end