function plot_surf(label, xdense, ydense, x,y,z)

    %The interpn function requires input data on a plaid grid, i.e. as you would produce with meshgrid, or perhaps ndgrid, 
    %while griddata (and scatteredInterpolant and TriScatteredInterp) expects input data on irregular grids. GRIDDATA expects the inputs  as 1-D vectors. 
    [XQ,YQ] = meshgrid(xdense,ydense);
    fun_interp = griddata(x,   y,   z,  XQ,YQ, 'cubic');
    figure
    surf(XQ,YQ,fun_interp, 'EdgeColor','k')
    xlabel("length", 'FontSize',20)
    ylabel("thetaf", 'FontSize',20)
    zlabel(label, 'FontSize',20)
    colormap(winter)
    colorbar
    xlim([min(xdense), max(xdense)])
    ylim([min(ydense), max(ydense)])
    view(-120, 28);
end
