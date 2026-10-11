# Explanation of the Sobel Operator

The sobel operator relies on a key assumption that a pixel that qualifies as an edge has a high rate of change of brightness. In more formal terms, if there is a function $I(x,y)$ which represents the brightness of a pixel at point $(x,y)$, then an edge would have a large $\Vert{}\nabla I\Vert{}$ while a non-edge would have $\nabla I \approx \mathbf{0}$.

The gradient is just the partial derivatives of each component vector of $I(x,y)$

$$\nabla I = \left( \frac{\partial I}{\partial x}, \frac{\partial I}{\partial y} \right)$$

