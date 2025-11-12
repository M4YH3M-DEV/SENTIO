# Affect.msg
# SENTIO unified emotion representation
# Valence-Arousal model with categorical label

# Valence: -1.0 (negative) to +1.0 (positive)
float32 valence

# Arousal: 0.0 (calm) to 1.0 (excited)
float32 arousal

# Categorical emotion label (integer code)
# 0: neutral, 1: happy, 2: sad, 3: angry, 4: fearful, 5: disgusted, 6: surprised
int32 label

# Human-readable label text (e.g., "vision_happy", "audio_neutral")
string label_text