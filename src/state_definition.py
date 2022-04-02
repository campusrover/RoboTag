# define states

#"wander around until polics is closer than x meters"...
Wander = 0 

#Escape, run directly away from the police
Run = 1

#"Chaser", "Police"...
Chase = 2

# 0 -> 1 if delta distance is less than 1m 
# 1-> 2 if delta distance is less than 10cm (assume 10cm is a touch/caught)
# 2-> 0 if you touched other robot 