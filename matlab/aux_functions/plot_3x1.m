function plot_3x1(x, y, title_, xlabel_, ylabel_, linetype, linewidth)

for i = 1:3
    subplot(3, 1, i);
    plot(x, y(i,:), linetype, 'LineWidth', linewidth);
    %ylabel(['$' ylabel_ '_' num2str(i) '}$'], 'interpreter', 'latex')    
    set(gca, 'FontName', 'Times New Roman');
    hold on;
end
xlabel(xlabel_, 'interpreter', 'latex');
title(title_);

subplot(3, 1, 2);
ylabel(['$' ylabel_ '$'], 'interpreter', 'latex');
end