from types import SimpleNamespace


def test_semantic_analyzer_handles_missing_optional_nodes():
    import compiler.ast as ast
    from compiler.semantical_errors import SemanticAnalyzer

    analyzer = SemanticAnalyzer(SimpleNamespace(), {}, {}, {})
    void_function = ast.FunctionNode(type=ast.VoidTypeNode(), name="demo")

    while_node = ast.WhileNode(expression=None, sentences=[])
    while_node.set_function(void_function)
    analyzer.visit_while(while_node, None)

    do_while_node = ast.DoWhileNode(expression=None, sentences=[])
    do_while_node.set_function(void_function)
    analyzer.visit_do_while(do_while_node, None)

    for_node = ast.ForNode(assignment=None, condition=None, expression=None, sentences=[])
    for_node.set_function(void_function)
    analyzer.visit_for(for_node, None)

    conditional = ast.ConditionalSentenceNode(condition=None, if_expr=[], else_expr=[])
    conditional.set_function(void_function)
    analyzer.visit_conditional_sentence(conditional, None)

    assignment = ast.AssignmentNode(var=None, expr=None)
    assignment.set_function(void_function)
    analyzer.visit_assignment(assignment, None)

    return_node = ast.ReturnNode(expression=None)
    return_node.set_function(void_function)
    analyzer.visit_return(return_node, None)

    inc_dec = ast.IncDecExpressionNode(var=None, op="++")
    inc_dec.set_function(void_function)
    analyzer.visit_inc_dec_expression(inc_dec, None)

    analyzer.visit_not_expression(ast.NotExpressionNode(expression=None), None)
    analyzer.visit_bit_not_expression(ast.BitNotExpressionNode(expression=None), None)
    analyzer.visit_arithmetic_expression(
        ast.ArithmeticExpressionNode(left=None, op="+", right=None),
        None,
    )
    analyzer.visit_comparision_expression(
        ast.ComparisionExpressionNode(left=None, op="==", right=None),
        None,
    )
    analyzer.visit_boolean_expression(
        ast.BooleanExpressionNode(left=None, op="&&", right=None),
        None,
    )
    analyzer.visit_bitwise_expression(
        ast.BitwiseExpressionNode(left=None, op="&", right=None),
        None,
    )
    analyzer.visit_compound_assignment(
        ast.CompoundAssignmentNode(left=None, op="+=", right=None),
        None,
    )

    assert analyzer._SemanticAnalyzer__get_declaration("missing", None) is None


def test_ast_builder_declaration_preserves_qualifiers():
    import compiler.ast as ast
    from compiler.ast_builder_visitor import ASTBuilderVisitor

    visitor = ASTBuilderVisitor()
    declaration = ast.DeclarationNode(type=ast.IntTypeNode(), var_name="value")
    visitor.visitSimple_declaration = lambda _ctx: declaration

    ctx = SimpleNamespace(
        declaration=lambda: None,
        s_def=object(),
        a_def=None,
        qual=SimpleNamespace(text="const"),
        start=SimpleNamespace(line=3, column=7),
    )

    result = visitor.visitDeclaration(ctx)

    assert result is declaration
    assert result.is_const is True
    assert result.line == 3
    assert result.position == 7


def test_standard_abs_uses_builtin_absolute_value():
    import libraries.standard as standard

    assert standard.abs(-12) == 12


def test_arduino_string_supports_index_access():
    import libraries.string as string_lib

    line = string_lib.String("+120")

    assert line[0] == "+"
    assert line[1] == "1"
    assert line[99] == "\0"


def test_stdlib_strtol_parses_values_and_updates_end_reference():
    import libraries.standard as standard
    import libraries.stdlib as stdlib

    end = standard.Ref()

    assert stdlib.strtol("ff", end, 16) == 255
    assert end.value == "\0"

    assert stdlib.strtol("not-a-number", end, 10) == 0
    assert end.value == "not-a-number"


def test_console_writes_main_thread_output_immediately():
    from output.console import Console

    class FakeText:
        def __init__(self):
            self.content = ""
            self.after_calls = []

        def tag_config(self, *_args, **_kwargs):
            return None

        def config(self, **_kwargs):
            return None

        def insert(self, _index, text, _tag=None):
            self.content += text

        def see(self, *_args):
            return None

        def after(self, delay, callback):
            self.after_calls.append((delay, callback))

    text = FakeText()
    console = Console(text)

    console.write_output("help\n")

    assert text.content == "help\n"
    assert text.after_calls == []


def test_arm3d_slider_sync_locks_manual_input_while_model_moves(monkeypatch):
    import graphics.gui as gui_mod
    import graphics.layers as layers_mod

    class DummySlider:
        def __init__(self):
            self.value = None
            self.options = {}
            self.on_set = None

        def grid(self):
            return None

        def grid_remove(self):
            return None

        def configure(self, **kwargs):
            self.options.update(kwargs)

        def set(self, value):
            if self.options.get("state") == "disabled":
                return
            self.value = value
            if self.on_set is not None:
                self.on_set(value)

    class DummyLabel:
        def __init__(self):
            self.text = ""

        def grid(self):
            return None

        def grid_remove(self):
            return None

        def config(self, **kwargs):
            self.text = kwargs.get("text", self.text)

    class DummyLayer:
        def __init__(self):
            self._moving = True
            self.motor3d = SimpleNamespace(
                model=SimpleNamespace(
                    dof=1,
                    joint_types=["R"],
                    joint_limits=[(-90.0, 90.0)],
                    joints=[10.0],
                )
            )

        def is_motion_active(self):
            return self._moving

        def _to_control_value(self, _idx, value):
            return value + 90.0

    monkeypatch.setattr(layers_mod, "Arm3DLayer", DummyLayer)

    calls = []
    layer = DummyLayer()
    slider = DummySlider()
    panel = gui_mod.Arm3DControlPanel.__new__(gui_mod.Arm3DControlPanel)
    panel.application = SimpleNamespace(
        controller=SimpleNamespace(
            robot_layer=layer,
            update_arm3d_joint=lambda idx, value: calls.append((idx, value)),
        )
    )
    panel._sliders = [slider]
    panel._val_labels = [DummyLabel()]
    panel._jlabels = [DummyLabel()]
    panel._syncing_sliders = False
    panel._sliders_locked = False
    slider.on_set = lambda value: gui_mod.Arm3DControlPanel._on_slider(panel, 0, value)

    gui_mod.Arm3DControlPanel.refresh_from_model(panel)

    assert slider.value == 100.0
    assert slider.options["state"] == "disabled"
    assert calls == []

    gui_mod.Arm3DControlPanel._on_slider(panel, 0, "120")
    assert calls == []

    layer._moving = False
    gui_mod.Arm3DControlPanel.refresh_from_model(panel)
    gui_mod.Arm3DControlPanel._on_slider(panel, 0, "120")

    assert slider.options["state"] == "normal"
    assert calls == [(0, 120.0)]
