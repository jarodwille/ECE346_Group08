function [pts, ij_pts] = readPoints(h, w, color, id)
    pts = zeros(2, 0);
    ij_pts = zeros(2, 0);

    num_lines = length(get(gca, 'children'));

    k = 0;
    
    while 1
        [i, j, but] = ginput(1);      % get a point
        if but == 2             % stop if click center button
            if k>0
                break
            else
                continue;
            end
        elseif but == 1 % left click add new point
            k = k + 1;
            [x,y] = ij2xy(i,j,w,h);
            pts(1,k) = x;
            pts(2,k) = y;
            ij_pts(1,k) = i;
            ij_pts(2,k) = j;
        elseif k>0 % right click delete last point;
            pts(:,k) = [];
            ij_pts(:,k) = [];
            k = k-1;
        else
            continue;
        end
        lines = get(gca, 'children');
        delete(lines(1:end-num_lines));
        plot(ij_pts(1,:), ij_pts(2,:), '-.o', 'LineWidth',3, ...
            'MarkerSize',12, 'Color',color, 'DisplayName',id);
    end
end

function [x,y]= ij2xy(i, j, w, h)
    L = 6.05;
    x = i/w*L;
    y = (1-j/h)*L;

end
