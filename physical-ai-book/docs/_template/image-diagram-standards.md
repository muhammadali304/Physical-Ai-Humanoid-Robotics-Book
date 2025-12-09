# Image and Diagram Standards for Educational Content

This document establishes standards for all visual elements in the Physical AI & Humanoid Robotics educational book to ensure consistency, accessibility, and educational effectiveness.

## Visual Style Guidelines

### Color Palette

#### Primary Colors
- **Primary Brand**: `#25c2a0` (Teal - used for highlights and important elements)
- **Secondary Brand**: `#3578e5` (Blue - used for links and secondary elements)
- **Accent Colors**:
  - Success: `#28a745` (Green)
  - Warning: `#ffc107` (Yellow)
  - Error: `#dc3545` (Red)
  - Info: `#17a2b8` (Cyan)

#### Accessibility Compliance
- All color combinations must meet WCAG 2.1 AA contrast requirements (minimum 4.5:1 for normal text, 3:1 for large text)
- Avoid color as the sole means of conveying information
- Test all visuals with color blindness simulators (protanopia, deuteranopia, tritanopia)

### Typography
- **Headings**: Sans-serif font (preferably system font stack)
- **Body text**: Sans-serif for digital, serif for print
- **Code elements**: Monospace font
- **Minimum size**: 14px for body text, 18px for headings
- **Line spacing**: 1.5x for readability

## Image Specifications

### Technical Requirements
- **Format**:
  - Diagrams: SVG (scalable vector graphics for crisp rendering)
  - Photographs: JPEG (85% quality) or PNG (for transparency)
  - Complex illustrations: PNG or SVG
- **Resolution**: Minimum 300 DPI for print, 72 DPI for web
- **File size**: Under 2MB per image, preferably under 500KB
- **Dimensions**: Maximum 1920px width for web display

### Image Structure
- **Padding**: Minimum 20px margin around content
- **Background**: White or transparent (avoid busy backgrounds)
- **Borders**: Use 1px solid `#ddd` for photographic images when needed
- **Shadows**: Subtle drop shadows (2px blur, 1px offset, 10% opacity) for emphasis

## Diagram Standards

### Architecture Diagrams
- **Boxes**: Rounded corners (4px radius), 2px solid border
- **Connectors**: Arrows for data flow, dashed lines for optional connections
- **Colors**: Use consistent color coding for different system components
- **Labels**: Clear, concise text with 12px minimum font size
- **Layout**: Hierarchical or flow-based depending on the concept

### Process Flow Diagrams
- **Start/End**: Rounded rectangles with green background
- **Process**: Rectangles with white background
- **Decision**: Diamonds with yellow background
- **Data**: Parallelograms
- **Flow**: Arrows with clear direction, labeled where ambiguous
- **Grid**: Use consistent spacing (20px minimum between elements)

### Sequence Diagrams
- **Actors**: Vertical dashed lines with activation boxes
- **Messages**: Horizontal arrows with clear labels
- **Time**: Top to bottom flow
- **Color coding**: Different colors for different message types (requests, responses, errors)

## Educational Design Principles

### Visual Hierarchy
1. **Primary**: Main concept or system component
2. **Secondary**: Supporting elements or connections
3. **Tertiary**: Context or background information
4. **Emphasis**: Use size, color, or contrast to highlight key elements

### Progressive Disclosure
- Start with simple diagrams showing core concepts
- Add complexity in subsequent diagrams
- Use callouts and annotations to explain details
- Provide zoomed views for complex elements

### Consistency
- Use the same visual elements for the same concepts throughout
- Maintain consistent color coding across all diagrams
- Follow the same layout patterns for similar types of information
- Use standardized icons and symbols

## Accessibility Standards

### Alt Text Requirements
- **Descriptive**: Explain the purpose and key information
- **Concise**: Under 125 characters when possible
- **Contextual**: Relate to the surrounding content
- **Actionable**: Help users understand the diagram's purpose

### SVG Accessibility
- Include `<title>` and `<desc>` elements
- Use meaningful `id` attributes for elements
- Include ARIA labels where appropriate
- Test with screen readers

### Color Accessibility
- Test all diagrams with color blindness simulators
- Provide text labels as backup for color-coded information
- Use patterns or textures in addition to color when needed
- Maintain high contrast ratios

## File Organization

### Directory Structure
```
static/
├── img/
│   ├── diagrams/
│   │   ├── ros2/
│   │   ├── simulation/
│   │   ├── isaac/
│   │   └── vla/
│   ├── screenshots/
│   │   ├── ros2/
│   │   ├── gazebo/
│   │   └── isaac_sim/
│   ├── photos/
│   │   ├── hardware/
│   │   └── demonstrations/
│   └── icons/
│       ├── navigation/
│       ├── status/
│       └── actions/
```

### Naming Convention
- Use lowercase with hyphens: `example-diagram-name.svg`
- Include descriptive prefixes: `arch-`, `flow-`, `seq-`, `photo-`
- Version when necessary: `example-diagram-name-v2.svg`
- Include chapter reference when helpful: `ch3-architecture-diagram.svg`

## Quality Assurance Checklist

Before including any image or diagram:

### Technical
- [ ] File format is appropriate for content type
- [ ] File size is optimized
- [ ] Resolution is adequate for intended use
- [ ] Colors meet accessibility contrast requirements
- [ ] SVG files include accessibility elements (when applicable)

### Educational
- [ ] Diagram supports the learning objectives
- [ ] Information is accurate and up-to-date
- [ ] Visual complexity matches the content difficulty
- [ ] Key concepts are clearly highlighted
- [ ] Labels and annotations are clear and readable

### Consistency
- [ ] Visual style matches other diagrams in the book
- [ ] Color coding is consistent with established standards
- [ ] Typography follows established guidelines
- [ ] Layout follows established patterns
- [ ] Terminology matches the text content

## Creation Tools and Templates

### Recommended Tools
- **Vector Graphics**: Inkscape (free), Adobe Illustrator (professional)
- **Diagrams**: Draw.io (free), Lucidchart (web-based), Visio (professional)
- **Screenshots**: Built-in OS tools or specialized capture software
- **Photos**: High-quality camera or professional photography

### Template Files
- Maintain template files for common diagram types
- Include standardized color palettes and fonts
- Provide grid and alignment guides
- Include accessibility elements in templates

## Examples

### Good Diagram Characteristics
- Clear visual hierarchy
- Consistent styling
- Appropriate level of detail
- Accessible color scheme
- Meaningful labels
- Clean layout with proper spacing

### Diagrams to Avoid
- Overly complex visualizations
- Low contrast color schemes
- Missing alt text or descriptions
- Inconsistent styling across the book
- Information-dense diagrams without progressive disclosure
- Decorative elements that distract from content

## Maintenance and Updates

### Version Control
- Keep source files (SVG, Draw.io, etc.) under version control
- Document changes and rationale
- Maintain backward compatibility when possible
- Test updated diagrams in the actual book context

### Review Process
- Peer review by technical experts
- Accessibility review by qualified personnel
- Student feedback incorporation
- Regular updates to maintain accuracy