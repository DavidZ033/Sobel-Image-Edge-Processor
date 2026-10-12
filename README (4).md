# Explanation of the Sobel Operator

The sobel operator relies on a key assumption that a pixel that qualifies as an edge has a high rate of change of brightness. In more formal terms, if there is a function $I(x,y)$ which represents the brightness of a pixel at point $(x,y)$, then an edge would have a large $\Vert{}\nabla I\Vert{}$ while a non-edge would have $\nabla I \approx \mathbf{0}$.

The gradient is just the partial derivatives of each component vector of $I(x,y)$

```math
\nabla I = \left( \frac{\partial I}{\partial x}, \frac{\partial I}{\partial y} \right)
```

Before we go into the actual process, we must explain what a convolution in imaging is. When we apply a 3x3 convolution to an image, the center of the matrix corresponds to the pixel and the numbers surrounding the pixel are coefficients multiplied by a value that corresponds to the pixels. Then all of thos evalues are summed together to get our new value. This matrix is called our kernel or filter. A visual example is shown below:

insert this later

You must apply two convolutions in order to detect an edge. One of these will smooth the image, and the other will estimate the partial derivative in a respective direction.

## Smoothing and Differentiation

More specifically, these smoothing and differentiation operations can be combined into a single kernel for each direction. The Sobel operator uses two of these combined kernels: one to estimate the horizontal derivative and another to estimate the vertical derivative. Also, because $I(x,y)$ is a scalar brightness function, its gradient contains its partial derivatives with respect to the two spatial coordinates.

Since an image consists of individual pixels, we estimate derivatives by comparing neighboring brightness values. For example, the horizontal derivative can be approximated by:

```math
\frac{\partial I}{\partial x}
\approx
\frac{I(x+1,y)-I(x-1,y)}{2}
```

Here, the distance between neighboring pixels is taken to be one unit. If the pixels to the left and right have similar brightness, the difference is small. If their brightness differs greatly, the difference is large.

Ignoring the constant factor of $1/2$ for now, we can represent this difference using:

```math
D = \begin{bmatrix} -1 & 0 & 1 \end{bmatrix}
```

However, comparing only two pixels makes the estimate sensitive to noise. To reduce this sensitivity, the Sobel operator also uses smoothing weights:

```math
S = \begin{bmatrix} 1 & 2 & 1 \end{bmatrix}
```

The center receives twice the weight of either neighboring position. Dividing by the sum of the weights, $4$, would give a weighted average. The usual Sobel kernels leave out this normalization.

## Constructing the Sobel Kernels

To estimate the horizontal derivative, we combine horizontal differentiation with vertical smoothing:

```math
K_x =
\begin{bmatrix}
1 \\ 2 \\ 1
\end{bmatrix}
\begin{bmatrix}
-1 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
-1 & 0 & 1 \\
-2 & 0 & 2 \\
-1 & 0 & 1
\end{bmatrix}
```

To estimate the vertical derivative, we combine vertical differentiation with horizontal smoothing:

```math
K_y =
\begin{bmatrix}
-1 \\ 0 \\ 1
\end{bmatrix}
\begin{bmatrix}
1 & 2 & 1
\end{bmatrix}
=
\begin{bmatrix}
-1 & -2 & -1 \\
0 & 0 & 0 \\
1 & 2 & 1
\end{bmatrix}
```

Here, $x$ increases to the right and $y$ increases downward, as is common for image coordinates.

Each kernel therefore performs both operations at once: it estimates a derivative in one direction while smoothing in the perpendicular direction.

## Applying the Kernels

Consider a pixel and its surrounding neighborhood:

```math
\begin{bmatrix}
a & b & c \\
d & e & f \\
g & h & i
\end{bmatrix}
```

The center pixel is $e$. Applying the horizontal kernel gives:

```math
G_x = (c + 2f + i) - (a + 2d + g)
```

This compares the brightness on the right with the brightness on the left. A large absolute value of $G_x$ indicates a strong horizontal change in brightness, which often corresponds to a vertical edge.

Applying the vertical kernel gives:

```math
G_y = (g + 2h + i) - (a + 2b + c)
```

This compares the brightness below the center with the brightness above it. A large absolute value of $G_y$ indicates a strong vertical change in brightness, which often corresponds to a horizontal edge.

Strictly speaking, multiplying and summing with the kernels exactly as shown is called cross-correlation. Mathematical convolution flips the kernels first, which reverses the signs of these responses but does not change the edge strength calculated below.

The values $G_x$ and $G_y$ are scaled estimates of the derivatives. Dividing each by $8$ accounts for both the central-difference factor of $2$ and the smoothing-weight sum of $4$. For edge detection, we can keep the unnormalized values as long as we use a consistent scale.

## Calculating Edge Strength

We combine the horizontal and vertical responses to calculate the gradient magnitude:

```math
G = \sqrt{G_x^2 + G_y^2}
```

A larger magnitude indicates a stronger local change in brightness, regardless of its direction.

For example, consider this neighborhood:

```math
\begin{bmatrix}
10 & 10 & 100 \\
10 & 10 & 100 \\
10 & 10 & 100
\end{bmatrix}
```

The horizontal response is:

```math
G_x = (100 + 2(100) + 100) - (10 + 2(10) + 10) = 360
```

The vertical response is:

```math
G_y = (10 + 2(10) + 100) - (10 + 2(10) + 100) = 0
```

Therefore:

```math
G = \sqrt{360^2 + 0^2} = 360
```

This matches what we would expect: the brightness changes from left to right, but not from top to bottom, indicating a vertical edge.

The result can exceed the original pixel brightness range because it represents a weighted difference rather than a pixel brightness.

## Calculating Edge Direction

The direction of the gradient can be calculated using:

```math
\theta = \mathrm{atan2}(G_y,G_x)
```

The function $\mathrm{atan2}$ uses the signs of both responses to determine the correct direction.

The gradient points toward the greatest increase in brightness, so it is perpendicular to the edge. For example, a vertical edge has a horizontal gradient.

## Creating an Edge Map

After calculating the gradient magnitude at each pixel, we have an image showing the strength of local brightness changes. To classify pixels as edges or non-edges, we can choose a threshold $T$:

```math
E(x,y) =
\begin{cases}
1, & G(x,y) > T \\
0, & G(x,y) \leq T
\end{cases}
```

A lower threshold detects weaker changes but may include more noise. A higher threshold keeps stronger changes but may miss faint edges.

Pixels at the image boundaries do not have a complete $3 \times 3$ neighborhood, so we must also choose how to handle them. Common approaches include extending the boundary values, padding the image, or skipping the outermost pixels.

The Sobel operator detects local changes in brightness, which may come from object boundaries, shadows, textures, or noise. Its output tells us where these changes occur and how strong they are, but it does not determine what caused them.