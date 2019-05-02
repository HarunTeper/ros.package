# ACTIONS = [(-0.4, 0.05), (0.4, 0.05), (-0.2, 0.1), (0.2, 0.1), (-0.1, 0.15), (0.1, 0.15)]
angle_space = np.linspace(start=-1, stop=1, num=18, endpoint=True)
velocity_space_half = np.concatenate((
        np.repeat(0.05, 6),
        np.repeat(0.1, 2),
        np.repeat(0.15, 1)))
velocity_space = np.concatenate(
    (velocity_space_half, 
    np.flip(velocity_space_half)))
   
ACTIONS = zip(angle_space, velocity_space)
ACTION_COUNT = len(ACTIONS)

print(velocity_space)
print(ACTION_COUNT)

LASER_SAMPLE_COUNT = 128  # Only use some of the LIDAR measurements

learning_rate = 0.001
gamma = 0.99
running_reward = 0
done = False
crash = False
deadlock = 0


class Policy(nn.Module):
    def __init__(self):
        super(Policy, self).__init__()
        self.state_space = LASER_SAMPLE_COUNT
        self.action_space = ACTION_COUNT
        
        self.l1 = nn.Linear(self.state_space, 64, bias=False)
        self.l2 = nn.Linear(64, 32, bias=False)
        self.l3 = nn.Linear(32, 16, bias=False)
        self.l4 = nn.Linear(16, 32, bias=False)
        self.l5 = nn.Linear(32, 64, bias=False)
        self.l6 = nn.Linear(64, self.state_space, bias=False)
        self.l7 = nn.Linear(self.state_space, self.action_space, bias=False)
        
        self.gamma = gamma
        
        # Episode policy and reward history 
        self.policy_history = Variable(torch.Tensor())
        self.reward_episode = []

        # Overall reward and loss history
        self.reward_history = []
        self.loss_history = []

    def forward(self, x):
        model = torch.nn.Sequential(
            self.l1,
            nn.Dropout(p=0.6),
            nn.ELU(),
            self.l2,
            nn.Dropout(p=0.6),
            nn.ELU(),
            self.l3,
            nn.Dropout(p=0.6),
            nn.ELU(),
            self.l4,
            nn.Dropout(p=0.6),
            nn.ELU(),
            self.l5,
            nn.Dropout(p=0.6),
            nn.ELU(),
            self.l6,
            nn.Dropout(p=0.6),
            nn.ELU(),
            self.l7,
            nn.Softmax(dim=-1)
        )
        return model(x)