function plot_3x3(x, y, title_, xlabel_, ylabel_, linetype, linewidth, ...
    font_size, desired)

if nargin < 8
    font_size = 10;
    desired = false;
elseif nargin < 9
    desired = false;
end

for i = 1:3
    for j = 1:3
        k = 3*(i - 1) + j;
        subplot(3, 3, k)
        
        if desired
            plot(x, squeeze(y(i,j,:)), linetype, ...
                'LineWidth', linewidth, 'Color', [1, 0, 0]);
        else
            plot(x, squeeze(y(i,j,:)), linetype, 'LineWidth', linewidth);
        end
        %ylabel(['$' ylabel_ '_' num2str(i) '}$'], 'interpreter', 'latex')    
        set(gca, 'FontName', 'Times New Roman', 'FontSize', font_size);
        ylim([-1 1]);
        hold on;
    end
end

title(title_);

subplot(3, 3, 8);
xlabel(xlabel_, 'interpreter', 'latex');

subplot(3, 3, 4);
ylabel(['$' ylabel_ '$'], 'interpreter', 'latex');
end