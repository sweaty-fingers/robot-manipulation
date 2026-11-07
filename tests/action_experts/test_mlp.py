import torch

from robot_manipulation.action_experts.mlp import MLPActionExpert


def test_predict_shape():
    expert = MLPActionExpert(input_dim=16, action_dim=3)
    obs_embed = torch.randn(4, 16)
    assert expert.predict(obs_embed).shape == (4, 3)


def test_compute_loss_decreases_with_training_step():
    expert = MLPActionExpert(input_dim=16, action_dim=3)
    obs_embed = torch.randn(8, 16)
    target = torch.randn(8, 3)
    optimizer = torch.optim.Adam(expert.parameters(), lr=1e-2)

    first_loss = expert.compute_loss(obs_embed, target)
    for _ in range(20):
        loss = expert.compute_loss(obs_embed, target)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
    last_loss = expert.compute_loss(obs_embed, target)

    assert last_loss.item() < first_loss.item()
