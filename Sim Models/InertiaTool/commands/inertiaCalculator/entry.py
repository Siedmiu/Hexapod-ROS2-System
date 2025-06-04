import adsk.core
import adsk.fusion
import traceback

# Global set to store our event handlers
handlers = []

def start():
    """
    Creates the command definition and adds it to the UI
    """
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        
        cmd_id = 'InertiaTool_InertiaCalculator'
        cmd_name = 'Inertia Calculator'
        cmd_description = 'Calculate moment of inertia matrix around custom axis with user-defined mass'
        
        cmd_def = ui.commandDefinitions.itemById(cmd_id)
        if not cmd_def:
            cmd_def = ui.commandDefinitions.addButtonDefinition(
                cmd_id, cmd_name, cmd_description, ''
            )

        command_created_handler = InertiaCalculatorCommandCreatedHandler()
        cmd_def.commandCreated.add(command_created_handler)
        handlers.append(command_created_handler)

        workspace = ui.workspaces.itemById('FusionSolidEnvironment')
        if workspace:
            panel = workspace.toolbarPanels.itemById('InspectPanel')
            if panel:
                control = panel.controls.itemById(cmd_id)
                if not control:
                    control = panel.controls.addCommand(cmd_def)
                    control.isVisible = True

    except Exception as e:
        app = adsk.core.Application.get()
        ui = app.userInterface
        ui.messageBox(f"Error starting Inertia Calculator: {str(e)}")


def stop():
    """
    Removes the command from the UI and cleans up event handlers
    """
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        cmd_id = 'InertiaTool_InertiaCalculator'
        workspace = ui.workspaces.itemById('FusionSolidEnvironment')
        if workspace:
            panel = workspace.toolbarPanels.itemById('InspectPanel')
            if panel:
                control = panel.controls.itemById(cmd_id)
                if control:
                    control.deleteMe()

        cmd_def = ui.commandDefinitions.itemById(cmd_id)
        if cmd_def:
            cmd_def.deleteMe()

    except Exception as e:
        app = adsk.core.Application.get()
        ui = app.userInterface
        ui.messageBox(f"Error stopping Inertia Calculator: {str(e)}")


class InertiaCalculatorCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    """
    Event handler for the command created event
    """
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            inputs = cmd.commandInputs

            # Create component selection input
            component_selection = inputs.addSelectionInput(
                'component_selection', 'Component', 
                'Select the component to calculate inertia for'
            )
            component_selection.setSelectionLimits(1, 1)

            # Create axis selection input
            axis_selection = inputs.addSelectionInput(
                'axis_selection', 'Reference Point', 
                'Select any point, edge, or coordinate system for reference'
            )
            axis_selection.setSelectionLimits(1, 1)

            # Create mass input
            mass_input = inputs.addFloatSpinnerCommandInput(
                'mass_input', 'Mass (grams)', '', 0.1, 10000.0, 0.1, 100.0
            )

            # Create calculate button
            inputs.addBoolValueInput('calculate_button', 'Calculate Inertia Matrix', False, '', False)
            
            # Create copy button
            copy_button = inputs.addBoolValueInput('copy_button', 'Copy Results to Clipboard', False, '', False)
            copy_button.isEnabled = False

            # Create text area for results
            results_text = inputs.addTextBoxCommandInput(
                'results_text', 'Results', '', 15, True
            )
            results_text.isFullWidth = True
            results_text.text = "Select a component and reference point, enter mass, then click Calculate."

            # Connect to input changed events
            input_changed_handler = InertiaCalculatorInputChangedHandler()
            cmd.inputChanged.add(input_changed_handler)
            handlers.append(input_changed_handler)

        except Exception as e:
            app = adsk.core.Application.get()
            ui = app.userInterface
            ui.messageBox(f"Error creating dialog: {str(e)}")


class InertiaCalculatorInputChangedHandler(adsk.core.InputChangedEventHandler):
    """
    Event handler for input changed events
    """
    def __init__(self):
        super().__init__()
        self.last_results = ""  # Store results for copying

    def notify(self, args):
        try:
            changed_input = args.input
            inputs = args.inputs

            # Handle calculate button
            if changed_input.id == 'calculate_button':
                if changed_input.value:
                    # Reset button
                    changed_input.value = False
                    
                    # Perform calculation
                    self.calculate_inertia(inputs)
                    
            # Handle copy button
            elif changed_input.id == 'copy_button':
                if changed_input.value:
                    changed_input.value = False
                    self.copy_to_clipboard()
            
            # Handle selection changes
            elif changed_input.id in ['component_selection', 'axis_selection']:
                # Selection inputs don't have .value attribute
                pass

        except Exception as e:
            app = adsk.core.Application.get()
            ui = app.userInterface
            ui.messageBox(f"Error handling input: {str(e)}")

    def calculate_inertia(self, inputs):
        """
        Calculate the moment of inertia matrix around the selected axis
        """
        try:
            # Get inputs
            component_selection = inputs.itemById('component_selection')
            axis_selection = inputs.itemById('axis_selection')
            mass_input = inputs.itemById('mass_input')
            results_text = inputs.itemById('results_text')
            copy_button = inputs.itemById('copy_button')

            if component_selection.selectionCount == 0:
                results_text.text = "Please select a component"
                copy_button.isEnabled = False
                return

            if axis_selection.selectionCount == 0:
                results_text.text = "Please select a reference point"
                copy_button.isEnabled = False
                return

            # Get selected component
            selected_entity = component_selection.selection(0).entity
            
            # Get component and its physical properties
            component = None
            if hasattr(selected_entity, 'component'):
                component = selected_entity.component
            elif hasattr(selected_entity, 'body'):
                component = selected_entity.body.parentComponent
            elif hasattr(selected_entity, 'parentComponent'):
                component = selected_entity.parentComponent
            else:
                results_text.text = "Unable to get component from selection. Try selecting a body or component."
                copy_button.isEnabled = False
                return

            # Calculate physical properties
            phys_props = component.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
            
            if not phys_props:
                results_text.text = "Unable to calculate physical properties. Make sure the component has a material assigned."
                copy_button.isEnabled = False
                return

            # Get center of mass (in component coordinates)
            center_of_mass = phys_props.centerOfMass
            
            # Get inertia tensor at center of mass (in kg⋅m²)
            inertia_cm = phys_props.getXYZMomentsOfInertia()
            fusion_mass = phys_props.mass  # in kg

            # Get user-defined mass in kg
            user_mass_kg = mass_input.value / 1000.0  # convert grams to kg

            # Calculate mass scaling factor
            mass_ratio = user_mass_kg / fusion_mass if fusion_mass > 0 else 1.0

            # Get reference point/axis
            reference_point = self.get_reference_point(axis_selection.selection(0).entity) #this has to be investigated
            
            if reference_point is None:
                results_text.text = "Unable to determine reference point from selection"
                copy_button.isEnabled = False
                return

            # Convert points to meters (Fusion works in cm)
            cm_x, cm_y, cm_z = center_of_mass.x / 100.0, center_of_mass.y / 100.0, center_of_mass.z / 100.0
            ref_x, ref_y, ref_z = reference_point.x / 100.0, reference_point.y / 100.0, reference_point.z / 100.0

            # Calculate displacement vector (from reference point to center of mass)
            dx = cm_x - ref_x
            dy = cm_y - ref_y  
            dz = cm_z - ref_z

            # Scale original inertia tensor by mass ratio
            Ixx_cm = inertia_cm[0] * mass_ratio
            Iyy_cm = inertia_cm[1] * mass_ratio
            Izz_cm = inertia_cm[2] * mass_ratio
            Ixy_cm = inertia_cm[3] * mass_ratio
            Ixz_cm = inertia_cm[4] * mass_ratio
            Iyz_cm = inertia_cm[5] * mass_ratio

            # Apply parallel axis theorem
            # I_new = I_cm + m * (d²*I - d⊗d)
            d_squared = dx*dx + dy*dy + dz*dz

            Ixx_new = Ixx_cm + user_mass_kg * (d_squared - dx*dx)
            Iyy_new = Iyy_cm + user_mass_kg * (d_squared - dy*dy)
            Izz_new = Izz_cm + user_mass_kg * (d_squared - dz*dz)
            Ixy_new = Ixy_cm - user_mass_kg * dx * dy
            Ixz_new = Ixz_cm - user_mass_kg * dx * dz
            Iyz_new = Iyz_cm - user_mass_kg * dy * dz

            # Format results
            results = self.format_results(
                component.name,
                user_mass_kg * 1000,  # back to grams
                fusion_mass * 1000,   # back to grams
                [cm_x*100, cm_y*100, cm_z*100],  # back to cm
                [ref_x*100, ref_y*100, ref_z*100],  # back to cm
                [[Ixx_new, Ixy_new, Ixz_new],
                 [Ixy_new, Iyy_new, Iyz_new],
                 [Ixz_new, Iyz_new, Izz_new]]
            )

            results_text.text = results
            self.last_results = results  # Store for copying
            copy_button.isEnabled = True  # Enable copy button

        except Exception as e:
            try:
                results_text = inputs.itemById('results_text')
                results_text.text = f"Error in calculation: {str(e)}"
                copy_button = inputs.itemById('copy_button')
                copy_button.isEnabled = False
            except:
                pass

    def copy_to_clipboard(self):
        """
        Copy results to clipboard using multiple methods with fallbacks
        """
        try:
            if self.last_results:
                clipboard_success = False
                
                # pyperclip should work but it requires the dependency
                try:
                    import pyperclip
                    pyperclip.copy(self.last_results)
                    clipboard_success = True
                    method_used = "pyperclip"
                except ImportError:
                    pass  # pyperclip not available
                except Exception:
                    pass  # pyperclip failed
                
                '''
                # I would love to use Fusion 360's clipboard but it doesn't work and doesn't throw exception
                # it returns a message that the characters were copied but they dont end up in the clipboard
                if not clipboard_success:
                    try:
                        app = adsk.core.Application.get()
                        app.clipboard = self.last_results
                        clipboard_success = True
                        method_used = "Fusion 360 clipboard"
                    except Exception:
                        pass
                '''
                
                # Show appropriate message
                app = adsk.core.Application.get()
                ui = app.userInterface
                
                if clipboard_success:
                    #cool
                    ui.messageBox(f"Results copied to clipboard!") #this is anoying
                else:
                    # Fallback: show results in a dialog for manual copy
                    # Truncate if too long for dialog
                    display_text = self.last_results
                    if len(display_text) > 2000:
                        display_text = display_text[:2000] + "\n\n... (truncated, full results are in the Results area above)"
                    
                    ui.messageBox(f"Automatic clipboard copy failed.\n\nPlease manually copy the results below:\n\n{display_text}")
            else:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("No results to copy. Calculate first.")
                
        except Exception as e:
            # Final fallback
            app = adsk.core.Application.get()
            ui = app.userInterface
            ui.messageBox(f"Copy function error: {str(e)}\n\nPlease manually copy from the Results area above.")

    def get_reference_point(self, entity):
        """
        Extract a reference point from the selected entity
        """
        try:
            if hasattr(entity, 'origin'):  # Coordinate system
                return entity.origin
            elif hasattr(entity, 'startVertex'):  # Edge
                return entity.startVertex.geometry
            elif hasattr(entity, 'geometry'):  # Point/Vertex
                return entity.geometry
            elif hasattr(entity, 'point'):  # Construction point
                return entity.point
            else:
                return None
        except:
            return None

    def format_results(self, component_name, user_mass, fusion_mass, center_of_mass, ref_point, inertia_matrix):
        """
        Format the calculation results for display
        """
        result = f"Component: {component_name}\n"
        result += f"User Mass: {user_mass:.1f} g\n"
        result += f"Fusion Mass: {fusion_mass:.1f} g\n"
        result += f"Mass Ratio: {user_mass/fusion_mass:.3f}\n\n"
        
        result += f"Center of Mass: ({center_of_mass[0]:.3f}, {center_of_mass[1]:.3f}, {center_of_mass[2]:.3f}) cm\n"
        result += f"Reference Point: ({ref_point[0]:.3f}, {ref_point[1]:.3f}, {ref_point[2]:.3f}) cm\n\n"
        
        result += "Inertia Matrix (kg⋅m²):\n"
        for i, row in enumerate(inertia_matrix):
            result += f"[{row[0]:9.6f} {row[1]:9.6f} {row[2]:9.6f}]\n"
        
        result += "\nInertia Matrix (g⋅cm²):\n"
        for i, row in enumerate(inertia_matrix):
            # Convert from kg⋅m² to g⋅cm²
            conv_row = [x * 1e7 for x in row]  # 1 kg⋅m² = 10^7 g⋅cm²
            result += f"[{conv_row[0]:12.3f} {conv_row[1]:12.3f} {conv_row[2]:12.3f}]\n"
        
        # Add matrix field reference
        result += "\nMatrix field reference:\n"
        result += "[Ixx Ixy Ixz]\n"
        result += "[Ixy Iyy Iyz]\n"
        result += "[Ixz Iyz Izz]\n"
        
        return result