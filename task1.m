clear; clc; close all;

load('RSdata_nocontrol.mat')
times = rt_estim.time(:);
xdata = rt_estim.signals.values(:,1);

 aircraft_state_array = rt_estim.signals.values(:,:);
    time = rt_estim.time(:);
    control_input_array = rt_motor.signals.values(:,:);
    fig = 1:1:4;
    col = 'b-';

PlotAircraftSim(time, aircraft_state_array ,control_input_array, fig, col)

function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
    s=1;
    e=3;
    y_label = ["x pos", "y pos", "z pos", "yaw", "pitch", "roll", "u", "v", "w", "p", "q", "r"];
    for j = 1:length(fig)
        figure(fig(j))
        for i = s:e
            sp = [1,2,3,1,2,3,1,2,3,1,2,3];
            subplot(3,1,sp(i))
            plot(time,aircraft_state_array(:,i),col)
            xlabel("Time (s)")
            ylabel(y_label(i));
        end
        s=s+3;
        e=e+3;
    end
    figure()
    subplot(4,1,1)
    plot(time,control_input_array(:,1),col); hold on;
    xlabel("Time (s)")
    ylabel("Z");
    subplot(4,1,2)
    plot(time,control_input_array(:,2),col); hold on;
    xlabel("Time (s)")
    ylabel("L");
    subplot(4,1,3)
    plot(time,control_input_array(:,3),col); hold on;
    xlabel("Time (s)")
    ylabel("M");
    subplot(4,1,4)
    plot(time,control_input_array(:,4),col); hold on;
    xlabel("Time (s)")
    ylabel("N");

    figure()
    plot3(aircraft_state_array(:,1),aircraft_state_array(:,2),-1*aircraft_state_array(:,3),col);
    hold on;
    plot3(aircraft_state_array(1,1), aircraft_state_array(1,2), -1*aircraft_state_array(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(aircraft_state_array(end,1), aircraft_state_array(end,2), -1*aircraft_state_array(end,3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    hold off;
end