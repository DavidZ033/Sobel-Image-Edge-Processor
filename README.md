# Explanation of the Sobel Operator

The sobel operator relies on a key assumption that a pixel that qualifies as an edge has a high rate of change of brightness. In more formal terms, if there is a function $I(x,y)$ which represents the brightness of a pixel at point $(x,y)$, then an edge would have a large $\Vert{}\nabla I\Vert{}$ while a non-edge would have $\nabla I \approx \mathbf{0}$.

The gradient is just the partial derivatives of each component vector of $I(x,y)$

$$\nabla I = \left( \frac{\partial I}{\partial x}, \frac{\partial I}{\partial y} \right)$$

Before we go into the actual process, we must explain what a convolution in imaging is. When we apply a 3x3 convolution to an image, the center of the matrix corresponds to the pixel and the numbers surrounding the pixel are coefficients multiplied by a value that corresponds to the pixels. Then all of thos evalues are summed together to get our new value. This matrix is called our kernel or filter. A visual example is shown below:

`[[10, 20, 30], [40, 50, 60], [70, 80, 90]]` $\ast$ `[[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]]` $\longrightarrow$ **Sum of Products** $\longrightarrow$ **80**

**Calculation Breakdown:**
* **Top Row:** $(-1 \times 10) + (0 \times 20) + (1 \times 30) = \mathbf{20}$
* **Middle Row:** $(-2 \times 40) + (0 \times 50) + (2 \times 60) = \mathbf{40}$
* **Bottom Row:** $(-1 \times 70) + (0 \times 80) + (1 \times 90) = \mathbf{20}$
* **Final Output Value:** $20 + 40 + 20 = \mathbf{80}$