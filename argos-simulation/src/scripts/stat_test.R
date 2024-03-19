# Script adapted from
# https://www.datanovia.com/en/lessons/mixed-anova-in-r/#two-way-mixed
# https://www.datanovia.com/en/lessons/repeated-measures-anova-in-r/

library(tidyverse)
library(ggpubr)
library(rstatix)

library(ez)

# Wide format
# set.seed(123)
my_data = read.csv(file='<path-to-file>/network_length.csv')
#my_data = read.csv(file='<path-to-file>/network_length_real_robot.csv')

# split the data by column 'Teams' and store them into separate variables as a dictionary
my_data_teams = split(my_data, my_data$Teams)

# [1] - 3 teams
# [2] - 4 teams
# [3] - 5 teams
# [4] - 6 teams
index = 1
team_data <- my_data_teams[[index]]
#print(team_data)

#team_data <- team_data[13:18,]
team_data <- team_data[, -1]
#team_data <- team_data[11:20,]

# For real robots
#team_data <- my_data

#print(team_data)

# Summary statistics
team_data %>%
  group_by(Solution) %>%
  get_summary_stats(Network.Length, type = "mean_sd")

# Visualization
bxp <- ggboxplot(
  team_data, x = "Solution", y = "Network.Length", add = "point", palette = "jco"
)
bxp

# Check assumptions

# Outliers
team_data %>%
  group_by(Solution) %>%
  identify_outliers(Network.Length)

# Normality assumption
team_data %>%
  group_by(Solution) %>%
  shapiro_test(Network.Length)

ggqqplot(team_data, "Network.Length", facet.by = "Solution")

# Homogneity of variance assumption
#my_data %>%
#  group_by(Condition) %>%
#  levene_test(Points ~ Communication)

# Homogeneity of covariances assumption
#box_m(my_data[, "Points", drop = FALSE], my_data$Communication)

# Two-way mixed ANOVA test
#res.aov <- anova_test(
#  data = my_data, dv = Points, wid = Users,
#  between = Communication, within = Condition
#)
#get_anova_table(res.aov)

# One-way repeated measures ANOVA test
#res.aov <- anova_test(
#  data = team_data, dv = Network.Length, wid = ID, within = Solution
#)
#get_anova_table(res.aov, correction = "GG")

# Conduct repeated measures ANOVA
anova_result <- ezANOVA(
  data = team_data,
  dv = .(Network.Length),
  wid=.(ID),
  within = .(Solution),
  detailed = FALSE
)

print(anova_result)

our_approach = team_data$Network.Length[team_data$Solution == "Our approach"]
steiner_tree = team_data$Network.Length[team_data$Solution == "Steiner tree"]
starlike = team_data$Network.Length[team_data$Solution == "Optimal starlike topology"]
#starlike = team_data$Network.Length[team_data$Solution == "Starlike topology"]
t1 <- t.test(our_approach,steiner_tree,paired=T)
t2 <- t.test(our_approach,starlike,paired=T)
pvalues <-c(t1$p.value, t2$p.value)
p.adjust(pvalues,'bonferroni',2)

cohens_d(team_data, Network.Length ~ Solution, paired = TRUE)

#model <- aov(Network.Length~factor(Solution)+Error(factor(ID)), data=team_data)
#summary(model)

#perform Tukey's Test
#TukeyHSD(model, conf.level=.95) 

#print(team_data)

# Post-hoc tests

# pairwise comparisons
pwc <- team_data %>%
  pairwise_t_test(
    Network.Length ~ Solution, paired = TRUE,
    p.adjust.method = "bonferroni"
  )
pwc



#Procedure for non-significant two-way interaction

# Within-subject
my_data %>%
  pairwise_wilcox_test(
    Points ~ Condition, paired = TRUE, 
    p.adjust.method = "bonferroni"
  )
# Between-subject
my_data %>%
  pairwise_wilcox_test(
    Points ~ Communication, 
    p.adjust.method = "bonferroni"
  )
