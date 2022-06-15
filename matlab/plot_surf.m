function plot_surf(label, xdense, ydense, x,y,z)

    [XQ,YQ] = meshgrid(xdense,ydense);
    fun_interp = griddata(x,   y,   z,  XQ,YQ);
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
