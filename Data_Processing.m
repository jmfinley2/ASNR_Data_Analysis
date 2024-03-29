%% Hands-on Tutorial for Data Processing Pipelines
%
% Although this session will focus on data analysis, many problems with
% rigor and reproducibility arise before any data are collected (e.g. study
% design and sampling).
%
%% General coding principles
% 
% * Write code for other humans and for your future self
% * Use descriptive variable names
% * Write good comments as you code. This requires striking a balance
%     between being comprehensive and readability. 
% * Create well-documented examples that demonstrate how your code works
% 
%% General steps in a data analysis pipeline
% From Leek and Peng, 2015, Nature. 
% <<Analysis_Pipeline.jpg>>

%% A pipeline for analzying data for a single participant (inner loop)
% # Import raw data
% # Perform manual or automated quality control (are measurements within
%     an expected range, can noise be filtered, can errors be fixed?)
% # Perform necessary pre-processing steps (segment long trials, compute
%     secondary variables of interest, filter data)
% # Perform manual or automated quality control
% # Compute summary measures (why averaging is important)
% # Perform manual or automated quality control
% # Save the results
% 
%% A pipeline for combining data from multiple participants (outer loop)
% # Load each participant's data
% # Store data in a single matrix or structurebz
% # Perform manual or automated quality control
% # Compute summary measures and perform statistical analysis
% # Generate figures
% 
%% Using simulated data to develop and validate analysis pipelines
%
% * *Take-home message #1*: Generating simulated data can help you build
% intuition about factors that may influence the variance in your
% measurements and reduce your statistical power. Thinking about these
% factors before you collect your data can help you improve the rigor of
% your study design. 
%
% * *Take-home message #2*: Most common statistical analyses can be performed
% using a similar, model-based analysis approach. 
% 
% * *Take-home message #3*: Your null hypotheses can generally be expressed as
% tests of whether coefficients in your model are equal to zero.
%
% The examples below are extensions of examples provided in the Data Skills
% for Reproducible Science online tutorial
% (https://psyteachr.github.io/msc-data-skills/) from the Richard Morey and
% Mossa Reimert at the University of Glasgow School of Psychology.
%
%% Example 1: Using simulations to build intuition about statistical power
%
% You are conducting a study to determine if peak shoulder flexion differs
% in a group that has had surgery to repair a labral tear and a group that
% received physical therapy for their labral tear, but no surgery.
% 
% Here, we will generate artificial/synthetic/simulated data for this study
% and examine how the estimated variability in our data influences the
% necessary sample size. 
%
%% Step 1: Specify estimates of the standard deviation in each group in degrees
SD_Surgical = 5; 
SD_Non_Surgical = 10;

%% Step 2: Specify the effect size of interest. 
% Here, effect size is defined as the mean difference between the groups
% divided by the pooled standard deviation. If the sample sizes are assumed
% to be equal, we will estimate the pooled standard deviation as the
% average standard deviation of each group. 
% 
% We will also assume that difference in peak flexion angle of 10 degrees
% is clinically meaningful. 
SD_Pooled = mean([SD_Surgical SD_Non_Surgical]);
MCID = 5;
Effect_Size = MCID/SD_Pooled;

%% Step 3: Use simulations to estimate statistical power for varying sample sizes
% Here, we will perform 1000 simulations for each of 20 different sample
% sizes ranging from five to 100 participants. For each simulation, we will
% draw a sample of participants from two normal distributions: one with a
% standard deviation equal SD_Surgical and another with a standard
% deviation equal to SD_Non_Surgical.
%
% We will then perform a t-test for each simulation and determine what
% fraction of these "experiments" returns a significant result assuming an
% alpha value of 0.05. Since we know that there is a true difference in the
% means of each sample, any time that we do not reject the null hypothesis
% is a false negative.
%
% *Congratulations, you're now doing Monte Carlo analysis!*
N_Simulations = 1000;
Sample_Size_Set = 5:5:100; 
N_Data_Points = numel(Sample_Size_Set);
Signif_Result = zeros(N_Simulations,N_Data_Points);
k = 1;

for Current_Sample_Size = Sample_Size_Set
    for i = 1:N_Simulations
        Data_Surgical = SD_Surgical*randn(Current_Sample_Size,1);
        Data_Non_Surgical = MCID + SD_Non_Surgical*randn(Current_Sample_Size,1);
        Signif_Result(i,k) = ttest2(Data_Surgical,Data_Non_Surgical);
    end
    k = k + 1;
end

Statistical_Power = 100*sum(Signif_Result)/N_Simulations;
ind = find(Statistical_Power > 80,1);

figure
plot(5:5:100,Statistical_Power, 'LineWidth',2,'Color','k'), xlabel('Sample Size')
ylabel('Statistical Power')
L = line([0 100],[80 80]); set(L,'LineWidth',2)
text(30,20,strcat(['You need a sample size of ~' num2str(Sample_Size_Set(ind)) ' for 80% power.']))

%% Example 2: Small samples generate misleading estimates of effect size
% Here, we will assume that we draw two samples from the same distribution.
% In this case, there is no true difference between the groups. 
%
Mean_Flexion_Angle = 100;
SD_Common = 10;
N_Simulations = 1000;
Sample_Size_Set = 1:100; 
N_Data_Points = numel(Sample_Size_Set);
Estimated_Effect_Size = zeros(N_Simulations,N_Data_Points);
k = 1;

for Current_Sample_Size = Sample_Size_Set
    for i = 1:N_Simulations
        Data_Surgical = Mean_Flexion_Angle + SD_Common*randn(Current_Sample_Size,1);
        Data_Non_Surgical = Mean_Flexion_Angle + SD_Common*randn(Current_Sample_Size,1);
        Estimated_Effect_Size(i,k) = abs((mean(Data_Surgical) - mean(Data_Non_Surgical))/mean([std(Data_Surgical) std(Data_Non_Surgical)]));
    end
    k = k + 1;
end

Median_Effect_Size = median(Estimated_Effect_Size);

figure
plot(Sample_Size_Set,Median_Effect_Size, 'LineWidth',2,'Color','k'), xlabel('Sample Size')
ylabel('Median Effect Size')
text(20,0.4,'Note that the true effect size is zero.')

%% Example 3: Potential problems with dichotomizing continuous variables
% Intervention studies are often interested in understanding the
% characteristics of a population that may make them more or less
% responsive to an intervention. 
%
% Here, we will show how splitting a continuous variable, in this case gait
% speed, into a dichotomous variable can produce misleading inferences
% about the characteristics of a study population that make them more or
% less responsive to an intervention. 
%
% We will simulate a pre/post evaluation of gait speed following
% a training intervention where there was *no effect*. To do so, we will
% generate a sample of 100 participants whose pre- and post-test gait speed
% are simply drawn at random from a normal distribution. 

Sample_Size = 100;
Mean_Gait_Speed = 0.5;
Lower_Bound = 0.2;
Upper_Bound = 0.8;
SD_Gait_Speed = 0.3;
Pre_Test_Speed = Mean_Gait_Speed + SD_Gait_Speed*randn(Sample_Size,1);
Pre_Test_Speed(Pre_Test_Speed < Lower_Bound) = Lower_Bound + (Lower_Bound - Pre_Test_Speed(Pre_Test_Speed < Lower_Bound));
Pre_Test_Speed(Pre_Test_Speed > Upper_Bound) = Upper_Bound - (Pre_Test_Speed(Pre_Test_Speed > Upper_Bound) - Upper_Bound);

Post_Test_Speed = (Mean_Gait_Speed + 0.2) + SD_Gait_Speed*randn(Sample_Size,1);

figure
subplot(1,2,1)
scatter(Pre_Test_Speed, Post_Test_Speed - Pre_Test_Speed)
xlabel('Baseline Speed (m/s)'), ylabel('Change in Speed (m/s)')

subplot(1,2,2)
Index_Slow = Pre_Test_Speed <= median(Pre_Test_Speed);
Index_Fast = Pre_Test_Speed > median(Pre_Test_Speed);
boxplot([Post_Test_Speed(Index_Slow) - Pre_Test_Speed(Index_Slow); Post_Test_Speed(Index_Fast) - Pre_Test_Speed(Index_Fast)],[zeros(sum(Index_Slow),1); ones(sum(Index_Fast),1)])
ylabel('Change in Speed (m/s)'), xlabel('Group'), set(gca,'XTickLabel',{'Slow' 'Fast'})

% Perform a t-test to determine if the "effects" of the intervention
% differed between groups.
[h, p] = ttest2(Post_Test_Speed(Index_Slow) - Pre_Test_Speed(Index_Slow),Post_Test_Speed(Index_Fast) - Pre_Test_Speed(Index_Fast))

%% Example 4: Using simulations to develop data analysis pipelines
% Assume that the peak should angle in the surgical group is *normally
% distributed* with a *mean of 100 degrees* and a *standard deviation of 5
% degrees*. We will also assume that the peak shoulder flexion angle in the
% non-surgical group is *normally distributed* with a *mean of 120 degrees*
% and a *standard deviation of 10 degrees*.
%
%
%% Step 1: Define your population parameters
Mean_Surgical = 100;        SD_Surgical = 5; 
Mean_Non_Surgical = 120;    SD_Non_Surgical = 10;

%% Step 2: Define your sample size for each group
Samp_Size = 50;

%% Step 3: Create a simulated dataset using the randn function. 

Peak_Shoulder_Angle_Surgical = Mean_Surgical + SD_Surgical*randn(Samp_Size,1);
Peak_Shoulder_Angle_Non_Surgical = Mean_Non_Surgical + SD_Non_Surgical*randn(Samp_Size,1);

%% Step 4: Sanity check #1: Visualize data 
% Make sure that the values are consistent with what you expect. 
figure, subplot(1,2,1), hist(Peak_Shoulder_Angle_Surgical)
xlabel('Shoulder Angle (deg)','FontSize',12)
ylabel('Count','FontSize',12)
title('Surgical Group')
xlim([80 150]), ylim([0 15]), set(gca,'FontSize',10), axis square

subplot(1,2,2), hist(Peak_Shoulder_Angle_Non_Surgical)
xlabel('Shoulder Angle (deg)','FontSize',12)
ylabel('Count','FontSize',12)
title('Non-Surgical Group')
xlim([80 150]), ylim([0 15]), axis square
set(gcf,'Units','centimeters','InnerPosition',[5 5 12 7])

%% Step 5: Sanity check #2: Compute summary statistics 
% Make sure that they are consistent with what you expect. 
Mean_Surgical_Sample = mean(Peak_Shoulder_Angle_Surgical);
SD_Surgical_Sample = std(Peak_Shoulder_Angle_Surgical);
subplot(1,2,1)
text(105,14,strcat(['Mean: ' num2str(Mean_Surgical_Sample,3)]))
text(105,12,strcat(['St Dev: ' num2str(SD_Surgical_Sample,3)]))

Mean_Non_Surgical_Sample = mean(Peak_Shoulder_Angle_Non_Surgical);
SD_Non_Surgical_Sample = std(Peak_Shoulder_Angle_Non_Surgical);
subplot(1,2,2)
text(82,14,strcat(['Mean: ' num2str(Mean_Non_Surgical_Sample,3)]))
text(82,12,strcat(['St Dev: ' num2str(SD_Non_Surgical_Sample,3)]))

%% Simplifying the alphabet soup of statistics with linear models
%
% Many of the most common statistical analysis procedures we use can be
% represented as a test of the coefficients of a linear model with
% continuous and categorical predictors.
% 
% *Example*: Use an independent-samples t-test to test the null hypothesis
% that your data are drawn from two normal distributions with the same
% mean. 
%
% *Translation*: Is there evidence to reject the claim that there is no
% difference in peak shoulder flexion between the surgical and therapy
% groups?
%
[Reject_Null, p_value] = ttest2(Peak_Shoulder_Angle_Surgical,Peak_Shoulder_Angle_Non_Surgical);
disp(p_value)

%% Alternative approach: Use a linear model 
% Treat group (Surgical/Rehab) as a categorical, independent variable and
% test whether the coefficients of the groups differ from one another.
% 
% $$\theta_{peak} = \beta_0 + \beta_1*Group$$
%
% For the Surgical Group, $Group = 0$. Therefore, $\beta_0$ is the estimated
% mean for the Surgical Group. 
%
% For the Non-Surgical group, $Group = 1$. Therefore, $\beta_0 + \beta_1$
% is the estimated mean for the Non-Surgical Group.
%
% If we reject the null hypothesis that $\beta_1 = 0$, this suggests that
% the means for each group differ from one another.
%
%%  Create a table to store your data. 
% Note that the third column of the Data table contains a "dummy variable"
% for Group assignment. This variable is zero for the Surgical group and
% one for the Non-Surgical group.
%
Data = table([Peak_Shoulder_Angle_Surgical; Peak_Shoulder_Angle_Non_Surgical],[zeros(50,1); ones(50,1)],...
    'VariableNames',{'Shoulder_Flex_Ang' 'Group'});
%% Fit a linear model and test if $\beta_1 = 0$ 
% *Note that the p-value for the Group effect is the same as the p-value
% from our t-test.*
%
% Also notice that the estimate of the Intercept, $\beta_0$ is equal to the
% mean peak shoulder flexion angle for the Surgical Group while the sum of
% the estimate of the Intercept and the Group effect, $\beta_1$ is equal to
% the mean peak should flexion angle for the Non-Surgical Group.
Model = fitlm(Data,'Shoulder_Flex_Ang ~ Group');
disp(Model)

